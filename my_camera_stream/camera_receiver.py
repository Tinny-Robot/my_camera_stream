#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraReceiver(Node):
    def __init__(self):
        super().__init__('camera_receiver')
        self.subscription = self.create_subscription(Image, 'camera_frame', self.callback, 10)
        self.bridge = CvBridge()

    def callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting ROS Image to OpenCV image: {e}")
            return

        # Display the image
        cv2.imshow("Camera Frame", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    camera_receiver = CameraReceiver()
    rclpy.spin(camera_receiver)
    camera_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
