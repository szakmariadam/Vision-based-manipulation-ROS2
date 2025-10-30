#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from dt_apriltags import Detector
import cv2
import os
from ament_index_python import get_package_share_directory
from cv_bridge import CvBridge

class ExtrinsicCalib(Node):
    def __init__(self):
        super().__init__("extrinsic_calib")
        self.get_logger().info("ExtrinsicCalib has started!")

        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.imageCallback,
            10
        )
        
        # Example timer (1 Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.tagFamily = "tagStandard41h12"

        self.square_size = 0.6

        self.bridge = CvBridge()
        self.frame = None

        self.tagCorners = np.array([
            [-self.square_size/2, self.square_size/2, 0],
            [self.square_size/2, self.square_size/2, 0],
            [self.square_size/2, -self.square_size/2, 0],
            [-self.square_size/2, -self.square_size/2, 0]], dtype=np.float32)
        
        #load intrinsics
        intrinsics_path = os.path.join(
                get_package_share_directory('sim_vision'),
                'config',
                'camera_calib.npz'
            )

        self.data = np.load(intrinsics_path)
        self.K = self.data['camera_matrix']
        self.dist = self.data['dist_coeffs']

        self.at_detector = Detector(
            families=self.tagFamily,
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25,
            debug=False,
        )

    def readImageGray(Self, image):
        if isinstance(image, str): #if image path
            img = cv2.imread(image)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        return gray
    
    def detectTags(self, gray):
        detection = self.at_detector.detect(gray)

        return detection
    
    def workspacePose(self, image):
        gray = self.readImageGray(image)

        detection = self.detectTags(gray)

        imgpoints = np.array(
            [
                detection[2].center,
                detection[3].center,
                detection[0].center,
                detection[1].center,    
            ])

        objpoints = self.tagCorners

        retval, rvec_workspace, tvec_workspace = cv2.solvePnP(objpoints, imgpoints, self.K, self.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)

        return rvec_workspace, tvec_workspace

    def cameraPose(self, rvec_workspace, tvec_workspace):
        # Convert rvec to rotation matrix
        R_workspace2cam, _ = cv2.Rodrigues(rvec_workspace)

        # Invert the rotation
        R_cam2workspace = R_workspace2cam.T

        # Invert the translation
        tvec_cam = -R_cam2workspace @ tvec_workspace

        # Convert rotation matrix back to rvec
        rvec_cam, _ = cv2.Rodrigues(R_cam2workspace)

        return rvec_cam, tvec_cam
    
    def timer_callback(self):
        rvecWorkspace, tvecWorkspace = self.workspacePose(self.frame)
        revecCam, tvecCam = self.cameraPose(rvecWorkspace, tvecWorkspace)

        self.get_logger().info("workspace pose found")
        self.get_logger().info(str(rvecWorkspace.ravel()))
        self.get_logger().info(str(tvecWorkspace.ravel()))

        self.get_logger().info("camera pose found")
        self.get_logger().info(str(revecCam.ravel()))
        self.get_logger().info(str(tvecCam.ravel()))

        #self.get_logger().info("No image recieved")

    def imageCallback(self, msg):
        # Convert ROS Image message to OpenCV image
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def main(args=None):
    rclpy.init(args=args)
    node = ExtrinsicCalib()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
