#!/usr/bin/env python3
import rclpy
from grasp_system.grasp_controller import GraspSystemController
from tests.conftest import create_mock_image
from cv_bridge import CvBridge

def main():
    rclpy.init()
    controller = GraspSystemController()
    bridge = CvBridge()
    
    mock_frame = create_mock_image()
    ros_msg = bridge.cv2_to_imgmsg(mock_frame, "bgr8")
    
    # Simulate a full pipeline pass
    controller.vision_servo.image_callback(ros_msg)
    controller.grasp_predictor.image_callback(ros_msg)
    # The controller's timer will then handle orchestration
    
    rclpy.spin_once(controller, timeout_sec=2.0)
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()