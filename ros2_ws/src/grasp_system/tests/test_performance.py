import pytest
from src.grasp_controller import GraspSystemController

def test_vision_servoing_latency():
    """Vision servoing should process at 30 FPS (33ms per frame)"""
    controller = GraspSystemController()
    mock_image = create_mock_image()
    
    with controller.perf_monitor.time_block('full_vision_cycle'):
        velocity = controller.vision_servo.process_frame(mock_image)
    
    latency = controller.perf_monitor.get_stats('full_vision_cycle')['mean']
    assert latency < 33.0, f"Vision latency {latency}ms exceeds 30 FPS requirement"

def test_grasp_prediction_latency():
    """Grasp prediction should run < 100ms for edge deployment"""
    controller = GraspSystemController()
    mock_image = create_mock_image()
    
    with controller.perf_monitor.time_block('grasp_inference'):
        quality = controller.grasp_predictor.predict(mock_image)
    
    latency = controller.perf_monitor.get_stats('grasp_inference')['mean']
    assert latency < 100.0, f"Grasp inference {latency}ms too slow for edge deployment"

def test_ik_solution_time():
    """IK solving should complete < 50ms"""
    controller = GraspSystemController()
    target_pose = [0.3, 0.2, 0.5, 0, 0, 0]
    
    with controller.perf_monitor.time_block('ik_solve'):
        solution = controller.ik_planner.solve_ik(target_pose)
    
    latency = controller.perf_monitor.get_stats('ik_solve')['mean']
    assert latency < 50.0, f"IK solving {latency}ms exceeds 50ms target"