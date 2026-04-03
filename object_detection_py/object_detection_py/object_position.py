import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from ament_index_python import get_package_share_directory
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener
import cv2
import ast
import os
import numpy as np

class ObjectPosition(Node):
    def __init__(self):
        super().__init__('object_position')

        self.bridge = CvBridge()

        self.bb_pos_subscription = Subscriber(
            self,
            Float64MultiArray,
            "/object_detection/bb_positions",
            5,
        )

        self.classes_subscription = Subscriber(
            self,
            String,
            "/object_detection/classes",
            5
        )

        self.obj_pos_publisher = self.create_publisher(
            Float64MultiArray,
            "/object_detection/obj_positions",
            5
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.bb_pos_subscription, self.classes_subscription],
            queue_size=10,
            slop=0.01,  # time tolerance (seconds)
            allow_headerless=True
        )

        self.sync.registerCallback(self.synced_callback)

        #get projection matrix
        intrinsics_path = os.path.join(
                get_package_share_directory('camera_vision_py'),
                'config',
                'camera_calib.npz'
            )
        data = np.load(intrinsics_path)
        self.K = data['camera_matrix']

        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(self.transform_buffer, self)

        self.get_logger().info("Object position node started.")

    def synced_callback(self, bb_pos, classes):
        classes_array = ast.literal_eval(classes.data)
        bb_pos_array = bb_pos.data
        try:
            camera_pose_quat = self.transform_buffer.lookup_transform("workspace_link", "camera_link_optical", rclpy.time.Time())
            camera_pose_quat_r = camera_pose_quat.transform.rotation
            camera_pose_quat_t = camera_pose_quat.transform.translation
            #self.get_logger().info(f'{camera_pose_quat_t.x}, {camera_pose_quat_t.y}, {camera_pose_quat_t.z}')
        except Exception as e:
            self.get_logger().error(str(e))
            return

        for i in range(0, len(classes_array)):
            bb_pos_array_i = np.array([])
            for j in range(0, 4):
                np.append(bb_pos_array_i, bb_pos_array[i*4+j])

            #self.get_logger().info(f'{classes_array[i]} bb pos: [{x1}, {y1}, {x2}, {y2}]')
            #get center in image
            if classes_array[i] == 'bottle':
                center_img = [int(bb_pos_array[0]+(bb_pos_array[2]-bb_pos_array[0])/2), int(bb_pos_array[3]-(bb_pos_array[3]-bb_pos_array[1])/7)]
                self.get_logger().info(f'{classes_array[i]} center: [{center_img[0]}, {center_img[1]}]')

def main(args=None):
    rclpy.init(args=args)

    node = ObjectPosition()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()