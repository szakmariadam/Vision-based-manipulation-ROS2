import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from cv_bridge import CvBridge
import cv2


class ObjectPosition(Node):
    def __init__(self):
        super().__init__('object_position')

        self.bridge = CvBridge()

        self.bb_pos_subscription = self.create_subscription(
            Float64MultiArray,
            '/object_detection/bb_positions',
            self.bb_pos_subscription_callback,
            5
        )

        self.classes_subscription = self.create_subscription(
            String,
            '/object_detection/classes',
            self.classes_subscription_callback,
            5
        )

        self.obj_pos_publisher = self.create_publisher(
            Float64MultiArray,
            "/object_detection/obj_positions",
            5
        )

        self.get_logger().info("Object position node started.")

    def bb_pos_subscription_callback(self, msg):
        self.bb_positions = msg

    def classes_subscription_callback(self, msg):
        self.classes = msg


def main(args=None):
    rclpy.init(args=args)

    node = ObjectPosition()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()