import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from object_detection.msg import ObjectDetection, ObjectPosition
from ament_index_python import get_package_share_directory
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
import cv2
import ast
import os
import numpy as np

class ObjectPositions(Node):
    def __init__(self):
        super().__init__('object_position')

        self.bridge = CvBridge()

        self.obj_det_sub = self.create_subscription(
            ObjectDetection,
            "/object_detection",
            self.obj_det_callback,
            5,
        )

        self.obj_pos_publisher = self.create_publisher(
            ObjectPosition,
            "/object_position",
            5
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

        #get projection matrix
        intrinsics_path = os.path.join(
                get_package_share_directory('camera_vision_py'),
                'config',
                'camera_calib.npz'
            )
        data = np.load(intrinsics_path)
        self.K = data['camera_matrix']
        self.dist = data['dist_coeffs']

        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(self.transform_buffer, self)

        self.classes_array = []
        self.bb_pos_array = []

        self.get_logger().info("Object position node started.")

    def obj_det_callback(self, msg):
        self.bb_pos_array = msg.bb_positions.data
        self.classes_array = ast.literal_eval(msg.classes.data)

    def timer_callback(self):
        try:
            camera_pose_quat = self.transform_buffer.lookup_transform("workspace_link", "camera_link_optical", rclpy.time.Time())
            camera_pose_quat_r = camera_pose_quat.transform.rotation
            camera_pose_t = camera_pose_quat.transform.translation
            #self.get_logger().info(f'{camera_pose_t.x}, {camera_pose_t.y}, {camera_pose_t.z}')
        except Exception as e:
            #self.get_logger().error(str(e))
            return

        r = Rotation.from_quat([camera_pose_quat_r.x, camera_pose_quat_r.y, camera_pose_quat_r.z, camera_pose_quat_r.w]).as_euler('xyz')
        R, _ = cv2.Rodrigues(r)
        t = np.array([camera_pose_t.x, camera_pose_t.y, camera_pose_t.z])

        object_positions = []
        class_names = []

        for i in range(0, len(self.classes_array)):
            #self.get_logger().info(f'{self.classes_array[i]} bb pos: [{self.bb_pos_array[i*4+0]}, {self.bb_pos_array[i*4+1]}, {self.bb_pos_array[i*4+2]}, {self.bb_pos_array[i*4+3]}]')

            #get bottom center in image
            center_img = [int(self.bb_pos_array[i*4+0]+(self.bb_pos_array[i*4+2]-self.bb_pos_array[i*4+0])/2), int(self.bb_pos_array[i*4+3])]
            #self.get_logger().info(f'{self.classes_array[i]} center: [{center_img[0]}, {center_img[1]}]')
            
            pts = np.array([[[center_img[0], center_img[1]]]], dtype=np.float32)
            undistorted = cv2.undistortPoints(pts, self.K, self.dist) #undistort points

            ray_cam = np.array([undistorted[0,0,0], undistorted[0,0,1], 1.0]) #direction in camera space
            ray_workspace = R @ ray_cam #transform ray direction to workpspace space
            ray_workspace /= np.linalg.norm(ray_workspace) #normalize
            origin = t #camera center (in workspace space)

            lambd = (0 - origin[2]) / ray_workspace[2] #solve lambda for intersection with ground plane (z=0)
            obj_pos = origin + lambd * ray_workspace #ray equation
            
            #get object center from object side
            #Only works from this angle!
            if self.classes_array[i] == 'bottle':
                obj_pos[1] = obj_pos[1] + 0.03 

            if self.classes_array[i] == 'cup':
                obj_pos[1] = obj_pos[1] + 0.06 

            #self.get_logger().info(f'{self.classes_array[i]} 3d pos: [{obj_pos[0]}, {obj_pos[1]}, {obj_pos[2]}]')
            class_names.append(self.classes_array[i])
            for pos in obj_pos: object_positions.append(pos)

        resultMsg = ObjectPosition()
        resultMsg.classes.data = str(class_names)
        resultMsg.obj_positions.data = object_positions
        self.obj_pos_publisher.publish(resultMsg)



def main(args=None):
    rclpy.init(args=args)

    node = ObjectPositions()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()