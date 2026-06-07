import pytest
import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge
from unittest.mock import MagicMock
import sys
import os

# Ensure Python can find your 'src' module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from src.vision_servoing import VisionServoController

@pytest.fixture(scope="module")
def ros_init():
    """Initializes and shuts down ROS 2 for the test session."""
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def node(ros_init):
    """Creates a fresh node for each test and mocks the publisher."""
    controller = VisionServoController()
    # Intercept the publish method so we can check what it tried to send
    controller.velocity_publisher.publish = MagicMock()
    yield controller
    controller.destroy_node()

def generate_mock_ros_image(center_x, center_y):
    """Creates a 640x480 mock camera frame with an ArUco marker at a specific X, Y."""
    frame = np.ones((480, 640), dtype=np.uint8) * 255
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(aruco_dict, 0, 100) # 100x100 pixel marker
    
    # Place marker based on desired center coordinates
    start_y, end_y = int(center_y - 50), int(center_y + 50)
    start_x, end_x = int(center_x - 50), int(center_x + 50)
    frame[start_y:end_y, start_x:end_x] = marker
    
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    bridge = CvBridge()
    return bridge.cv2_to_imgmsg(frame_bgr, "bgr8")

def test_feature_detection_and_error(node):
    """Test: Feature detection & Error computation (expected vs actual)"""
    msg = generate_mock_ros_image(420, 240)
    node.image_callback(msg)
    
    # Use pytest.approx to allow for a 1-pixel tolerance in computer vision math
    assert np.median(node.error_history_x) == pytest.approx(-100.0, abs=1.0)
    assert np.median(node.error_history_y) == pytest.approx(0.0, abs=1.0)

def test_control_output_and_saturation(node):
    """Test: Control output within expected range & Velocity saturation"""
    msg = generate_mock_ros_image(60, 240)
    
    node.image_callback(msg)
    
    node.velocity_publisher.publish.assert_called_once()
    published_msg = node.velocity_publisher.publish.call_args[0][0]
    
    assert published_msg.angular.z == 0.5
    # Allow a tiny tolerance for subpixel math on the Y axis
    assert published_msg.linear.z == pytest.approx(0.0, abs=0.01)
    

def test_marker_lost_stops_robot(node):
    """Test: Edge case where marker is missing (velocity should be 0)"""
    # Create a completely blank white image (no marker)
    frame = np.ones((480, 640, 3), dtype=np.uint8) * 255
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    
    node.image_callback(msg)
    
    node.velocity_publisher.publish.assert_called_once()
    published_msg = node.velocity_publisher.publish.call_args[0][0]
    
    # Verify velocities are defaulted to 0.0
    assert published_msg.angular.z == 0.0
    assert published_msg.linear.z == 0.0