import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import cv2


class DetectObject(Node):
    def __init__(self):
        super().__init__('detect_object')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            CompressedImage,
            '/table_camera/image/compressed',
            self.image_callback,
            20
        )

        self.get_logger().info("Object detection node started.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

            #ide közé az object detection

            # Show image
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = DetectObject()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()