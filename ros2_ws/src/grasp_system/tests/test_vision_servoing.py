import pytest
import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge
from unittest.mock import MagicMock
import sys
import os

# Ensure Python can find your module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from grasp_system.vision_servoing import VisionServoController

# The fixture for node now explicitly requires the 'ros_init' fixture from conftest.py
@pytest.fixture
def node(ros_init):
    """Creates a fresh node for each test and mocks the publisher."""
    controller = VisionServoController()
    # Intercept the publish method so we can check what it tried to send
    controller.velocity_publisher.publish = MagicMock()
    yield controller
    controller.destroy_node()

def generate_mock_ros_image(center_x, center_y):
    """Creates a 640x480 mock camera frame with an ArUco marker."""
    frame = np.ones((480, 640), dtype=np.uint8) * 255
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    # Generate marker
    marker = cv2.aruco.generateImageMarker(aruco_dict, 0, 100) 
    
    # Place marker
    start_y, end_y = int(center_y - 50), int(center_y + 50)
    start_x, end_x = int(center_x - 50), int(center_x + 50)
    frame[start_y:end_y, start_x:end_x] = marker
    
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    bridge = CvBridge()
    return bridge.cv2_to_imgmsg(frame_bgr, "bgr8")

def test_feature_detection_and_error(node):
    """Test: Feature detection & Error computation"""
    msg = generate_mock_ros_image(420, 240)
    node.image_callback(msg)
    
    # Asserting error history calculations
    assert np.median(node.error_history_x) == pytest.approx(-100.0, abs=1.0)
    assert np.median(node.error_history_y) == pytest.approx(0.0, abs=1.0)

def test_control_output_and_saturation(node):
    """Test: Control output within expected range & Velocity saturation"""
    msg = generate_mock_ros_image(60, 240)
    node.image_callback(msg)
    
    node.velocity_publisher.publish.assert_called_once()
    published_msg = node.velocity_publisher.publish.call_args[0][0]
    
    # Verify velocity output
    assert published_msg.angular.z == pytest.approx(0.5, abs=0.1)
    assert published_msg.linear.z == pytest.approx(0.0, abs=0.01)

def test_marker_lost_stops_robot(node):
    """Test: Marker missing (velocity should be 0)"""
    frame = np.ones((480, 640, 3), dtype=np.uint8) * 255
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    
    node.image_callback(msg)
    
    node.velocity_publisher.publish.assert_called_once()
    published_msg = node.velocity_publisher.publish.call_args[0][0]
    
    assert published_msg.angular.z == 0.0
    assert published_msg.linear.z == 0.0