import numpy as np

def create_mock_image():
    """Create a dummy 480x640 BGR image for testing."""
    return np.zeros((480, 640, 3), dtype=np.uint8)