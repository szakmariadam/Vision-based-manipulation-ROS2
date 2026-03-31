import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
import cv2
import ast

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

        self.get_logger().info("Object position node started.")

    def synced_callback(self, bb_pos, classes):
        classes_array = ast.literal_eval(classes.data)
        bb_pos_array = bb_pos.data

        for i in range(0, len(classes_array)):
            x1 = bb_pos_array[i*4+0]
            y1 = bb_pos_array[i*4+1]
            x2 = bb_pos_array[i*4+2]
            y2 = bb_pos_array[i*4+3]

            self.get_logger().info(f'{classes_array[i]} bb pos: [{x1}, {y1}, {x2}, {y2}]')


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