#!/usr/bin/env python3
import rclpy
from grasp_system.vision_servoing import VisionServoController
from tests.conftest import create_mock_image
from cv_bridge import CvBridge

def main():
    rclpy.init()
    node = VisionServoController()
    bridge = CvBridge()
    
    # Process a single mock frame
    mock_frame = create_mock_image()
    ros_msg = bridge.cv2_to_imgmsg(mock_frame, "bgr8")
    
    node.get_logger().info("Running vision loop...")
    node.image_callback(ros_msg)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()