import pytest
import rclpy
import numpy as np
import torch
import sys
import os

# Ensure Python can find your 'src' module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from grasp_system.grasp_predictor import GraspQualityPredictor

@pytest.fixture(scope="module")
def ros_init():
    """Initializes and shuts down ROS 2 for the test session."""
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def node(ros_init):
    """Creates a fresh node for each test."""
    predictor = GraspQualityPredictor()
    yield predictor
    predictor.destroy_node()

def test_model_loads_correctly(node):
    """Test: Model loads correctly"""
    # The model should not be None and should be a valid PyTorch module
    assert node.model is not None
    assert isinstance(node.model, torch.nn.Module)

def test_inference_output_in_valid_range(node):
    """Test: Inference output in valid range [0-1]"""
    # Create a valid 3-channel dummy image (BGR format like OpenCV)
    dummy_image = np.ones((480, 640, 3), dtype=np.uint8) * 128
    
    score, candidates = node.predict_grasp_quality(dummy_image)
    
    # Probability must be bounded between 0.0 and 1.0 (Sigmoid output)
    assert 0.0 <= score <= 1.0
    
    # Ensure it returns the expected candidate structure
    assert len(candidates) > 0
    for cand in candidates:
        assert 'x' in cand and 'y' in cand and 'score' in cand

def test_handles_invalid_inputs_gracefully(node):
    """Test: Handles invalid inputs gracefully (no crashing)"""
    # Pass 'None' instead of an image array
    score, candidates = node.predict_grasp_quality(None)
    
    # It should catch the error and return safe default values
    assert score == 0.0
    assert candidates == []

def test_preprocessing_pipeline(node):
    """Test: Preprocessing (image resizing, normalization)"""
    # Create a giant, blank image
    dummy_image = np.ones((1000, 1000, 3), dtype=np.uint8) * 255
    
    # Run it purely through the PyTorch transforms
    tensor = node.preprocess(dummy_image)
    
    # Verify the tensor was resized exactly to MobileNetV2 standards:
    # 3 Color Channels, 224 Height, 224 Width
    assert tensor.shape == (3, 224, 224)
    # Verify it was converted to a float tensor for the neural net
    assert tensor.dtype == torch.float32