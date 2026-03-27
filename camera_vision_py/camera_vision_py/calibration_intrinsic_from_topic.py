import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import numpy as np
import os


class CameraIntrinsicsSaver(Node):

    def __init__(self):
        super().__init__('camera_intrinsics_saver')

        # Parameters
        self.declare_parameter('topic', '/table_camera/camera_info')
        self.declare_parameter('output_file', 'camera_calib.npz')
        self.declare_parameter('save_once', True)

        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.save_once = self.get_parameter('save_once').get_parameter_value().bool_value

        self.subscription = self.create_subscription(
            CameraInfo,
            topic,
            self.camera_info_callback,
            10
        )

        self.saved = False

        self.get_logger().info(f"Listening to {topic}...")

    def camera_info_callback(self, msg: CameraInfo):
        if self.save_once and self.saved:
            return

        self.get_logger().info("Received CameraInfo message, saving intrinsics to NPZ...")

        # Convert to numpy arrays
        K = np.array(msg.k).reshape(3, 3)
        D = np.array(msg.d)
        R = np.array(msg.r).reshape(3, 3)
        P = np.array(msg.p).reshape(3, 4)

        # Ensure directory exists
        os.makedirs(os.path.dirname(self.output_file) or '.', exist_ok=True)

        # Save to npz
        np.savez(
            self.output_file,
            image_width=msg.width,
            image_height=msg.height,
            distortion_model=msg.distortion_model,
            camera_matrix=K,
            dist_coeffs=D,
            R=R,
            P=P
        )

        self.get_logger().info(f"Intrinsics saved to {self.output_file}")

        self.saved = True

        if self.save_once:
            self.get_logger().info("Shutting down after saving once.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CameraIntrinsicsSaver()
    rclpy.spin(node)


if __name__ == '__main__':
    main()