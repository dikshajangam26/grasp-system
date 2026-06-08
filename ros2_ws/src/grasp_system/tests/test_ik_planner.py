import pytest
import rclpy
import numpy as np
import sys
import os

# Ensure Python can find your 'src' module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from grasp_system.ik_planner import InverseKinematicsSolver

@pytest.fixture
def node(ros_init):
    """Creates a fresh IK node for each test."""
    solver = InverseKinematicsSolver()
    yield solver
    solver.destroy_node()

def test_ik_solution_within_joint_limits(node):
    """Test: IK solution within joint limits"""
    # A known safe coordinate directly in front of the robot
    safe_target = [0.32, 0.24, 0.5, 0.0, 3.14, 0.0] 
    
    joint_angles = node.solve_ik(safe_target)
    
    # Verify the solver found a solution and output exactly 6 angles
    assert joint_angles is not None
    assert len(joint_angles) == 6
    
    # Verify the solver mathematically confirmed it does not violate our constraints
    assert node.validate_constraints(joint_angles) is True

def test_workspace_validation(node):
    """Test: Workspace validation (reachable/unreachable poses)"""
    # Target is 2.0 meters away. Our robot's max reach is ~1.1 meters.
    unreachable_target = [2.0, 0.0, 0.0] 
    
    joint_angles = node.solve_ik(unreachable_target)
    
    # The solver MUST reject this and return None instead of breaking the arm
    assert joint_angles is None

def test_invalid_inputs_handled_gracefully(node):
    """Test: Invalid inputs handled gracefully (no crashing)"""
    # Pass 'None' instead of a [x, y, z] coordinate list
    joint_angles = node.solve_ik(None)
    
    # The exception block should catch this and return safely
    assert joint_angles is None

def test_trajectory_generated_with_correct_waypoints(node):
    """Test: Trajectory generated with correct number of waypoints"""
    # Mock a target array of safe joint angles
    target_angles = np.array([0.5, 0.5, 0.5, 0.0, 0.0, 0.0]) 
    
    trajectory = node.plan_trajectory(target_angles)
    
    assert trajectory is not None
    # We hardcoded num_waypoints = 10 in our path planner
    assert len(trajectory) == 10