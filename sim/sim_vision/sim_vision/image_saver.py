#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from datetime import datetime

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.frame = None
        self.get_logger().info("ImageSaver node started. Press 's' to save the current frame, or 'q' to quit.")

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.frame is not None:
                cv2.imshow("Camera Feed", self.frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('s'):
                    self.save_image()
                elif key == ord('q'):
                    self.get_logger().info("Quitting...")
                    break
        cv2.destroyAllWindows()

    def save_image(self):
        if self.frame is None:
            self.get_logger().warn("No frame received yet!")
            return
        filename = f"image_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        cv2.imwrite(filename, self.frame)
        self.get_logger().info(f"Saved image as {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
