#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sys
import termios
import tty


class KeyboardNode(Node):

    def __init__(self):
        super().__init__('keyboard_node')

        self.publisher_ = self.create_publisher(String, 'object_manipulation', 10)

        self.get_logger().info("Object manipulation controller started. Press q to quit...")
        self.get_logger().info("Press 'b' for bottle")
        self.get_logger().info("Press 'c' for cup")
        self.get_logger().info("Press 't' for tennis ball")

    def get_key(self):

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        return key

    def run(self):
        while rclpy.ok():
            key = self.get_key()

            msg = String()

            if key == 'b':
                self.get_logger().info("Getting bottle...")
                msg.data = "bottle"
                self.publisher_.publish(msg)

            elif key == 'c':
                self.get_logger().info("Getting cup...")
                msg.data = "cup"
                self.publisher_.publish(msg)

            elif key == 't':
                self.get_logger().info("Getting tennis ball...")
                msg.data = "sports ball"
                self.publisher_.publish(msg)                

            elif key == 'q':
                self.get_logger().info("Quitting...")
                break


def main(args=None):
    rclpy.init(args=args)

    node = KeyboardNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()