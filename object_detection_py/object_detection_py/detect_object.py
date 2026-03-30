import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Float64MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np


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

        self.classes_publisher = self.create_publisher(
            String,
            "/object_detection/classes",
            5
        )

        self.bb_pos_publisher = self.create_publisher(
            Float64MultiArray,
            "/object_detection/bb_positions",
            5
        )

        self.image_annoted_publisher = self.create_publisher(
            Image,
            "/object_detection/image_annoted",
            5
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.model = YOLO("yolo26n.pt")

        self.get_logger().info("Object detection node started.")

    def timer_callback(self):
        try:
            det_result = self.model(self.image)
            det_annoted = det_result[0].plot()

            classes = det_result[0].boxes.cls.cpu().numpy().astype(int)
            class_names = [det_result[0].names[i] for i in classes]

            bb_positions = det_result[0].boxes.xyxy.cpu().numpy()

            bb_positions_flat = np.concatenate(bb_positions)

            self.classes_publisher.publish(String(data=str(class_names)))
            self.bb_pos_publisher.publish(Float64MultiArray(data=bb_positions_flat))

            #publish image
            img_msg = self.bridge.cv2_to_imgmsg(det_annoted, encoding="bgr8")
            self.image_annoted_publisher.publish(img_msg)

            # Show image
            #cv2.imshow("Camera Feed", det_annoted)
            #cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")
        

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        self.image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')


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