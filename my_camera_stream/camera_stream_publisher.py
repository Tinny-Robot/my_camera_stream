#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraStreamPublisher(Node):
    def __init__(self):
        super().__init__('camera_stream_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_frame', 10)
        self.timer_ = self.create_timer(1.0 / 30, self.publish_frame)
        self.bridge = CvBridge()

    def publish_frame(self):
        cap = cv2.VideoCapture(0)  # Use 0 for default camera or change to the appropriate camera index
        ret, frame = cap.read()
        cap.release()

        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(img_msg)

def main(args=None):
    
    rclpy.init(args=args)
    camera_stream_publisher = CameraStreamPublisher()
    rclpy.spin(camera_stream_publisher)
    camera_stream_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()