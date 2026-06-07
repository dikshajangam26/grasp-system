#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class MockCamera(Node):
    def __init__(self):
        super().__init__('mock_camera')
        # Publish to the exact topic your servo node is listening to
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Publish an image every 0.5 seconds (2 Hz)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.bridge = CvBridge()
        
        # Generate a mathematical, perfect ArUco marker (200x200 pixels)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.marker_image = cv2.aruco.drawMarker(self.aruco_dict, 0, 200)        
        self.get_logger().info('Mock Camera initialized. Publishing fake frames...')

    def timer_callback(self):
        # 1. Create a blank white image (Resolution: 640 width x 480 height)
        frame = np.ones((480, 640), dtype=np.uint8) * 255
        
        # 2. Place the marker off-center to generate an intentional error
        # Center of frame is X=320, Y=240. We will place the marker at X=400, Y=300
        start_y, end_y = 200, 400
        start_x, end_x = 300, 500
        frame[start_y:end_y, start_x:end_x] = self.marker_image
        
        # 3. Convert grayscale to standard color (BGR) format
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        
        # 4. Convert OpenCV image to ROS 2 Image message and publish
        msg = self.bridge.cv2_to_imgmsg(frame_bgr, "bgr8")
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()