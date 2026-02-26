import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class IPCameraPublisher(Node):
    def __init__(self):
        super().__init__('ip_camera_publisher')

        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture("https://192.168.0.95:8080/video")

        timer_period = 0.03  # ~30 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IPCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()