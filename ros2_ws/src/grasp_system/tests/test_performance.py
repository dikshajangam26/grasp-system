import pytest
from grasp_system.grasp_controller import GraspSystemController
from tests.conftest import create_mock_image

def test_vision_servoing_latency():
    """Vision servoing should process at 30 FPS (33ms per frame)"""
    controller = GraspSystemController()
    mock_image = create_mock_image()
    
    # Target the sub-node's monitor
    # Note: Ensure your vision node uses the name 'vision_detection' in its time_block
    controller.vision_servo.image_callback(mock_image)
    
    stats = controller.vision_servo.perf_monitor.get_stats('vision_detection')
    latency = stats['mean']
    
    assert latency < 33.0, f"Vision latency {latency:.2f}ms exceeds 30 FPS requirement"
    controller.destroy_node()

def test_grasp_prediction_latency():
    """Grasp prediction should run < 100ms for edge deployment"""
    controller = GraspSystemController()
    mock_image = create_mock_image()
    
    # Target the predictor's monitor
    controller.grasp_predictor.image_callback(mock_image)
    
    stats = controller.grasp_predictor.perf_monitor.get_stats('grasp_inference')
    latency = stats['mean']
    
    assert latency < 100.0, f"Grasp inference {latency:.2f}ms too slow for edge deployment"
    controller.destroy_node()

def test_ik_solution_time():
    """IK solving should complete < 50ms"""
    controller = GraspSystemController()
    target_pose = [0.3, 0.2, 0.5, 0, 0, 0]
    
    # Target the IK planner's monitor
    controller.ik_planner.solve_ik(target_pose)
    
    stats = controller.ik_planner.perf_monitor.get_stats('ik_solving')
    latency = stats['mean']
    
    assert latency < 50.0, f"IK solving {latency:.2f}ms exceeds 50ms target"
    controller.destroy_node()