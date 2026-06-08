import numpy as np
import pytest
import rclpy

def create_mock_image():
    """Create a dummy 480x640 BGR image for testing."""
    return np.zeros((480, 640, 3), dtype=np.uint8)

@pytest.fixture(scope="session", autouse=True)
def ros_init():
    if not rclpy.ok():
        rclpy.init()
    yield
