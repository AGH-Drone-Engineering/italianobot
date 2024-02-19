#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv


class ImagePublisher(Node):

    def __init__(self):
        super().__init__("camera_node")
        self.image_publisher = self.create_publisher(
            Image, "/camera/super", 10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv.VideoCapture(0)
        self.cv_bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret == True:
            self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(frame, "bgr8"))

        self.get_logger().info("Publishing video")


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
