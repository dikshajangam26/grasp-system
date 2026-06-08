#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Float32MultiArray
from geometry_msgs.msg import Twist
import yaml
from pathlib import Path

# Importing the nodes directly from the package
from grasp_system.vision_servoing import VisionServoController
from grasp_system.grasp_predictor import GraspQualityPredictor
from grasp_system.ik_planner import InverseKinematicsSolver

class GraspSystemController(Node):
    def __init__(self):
        super().__init__('grasp_system_controller')
        
        # 1. Load System Configuration
        # Navigate up 4 levels: grasp_controller.py -> grasp_system -> grasp_system -> src -> ros2_ws -> grasp-system
        repo_root = Path(__file__).resolve().parents[4]
        config_file = repo_root / 'config' / 'system_config.yaml'
        
        try:
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
            self.get_logger().info(f"Successfully loaded configuration from {config_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to load config at {config_file}. Error: {e}")
            self.config = {} # Fallback to empty dict so the code doesn't crash
        
        # 2. Initialize sub-modules
        # If you update your sub-modules to accept parameters, you can pass them here like:
        # self.vision_servo = VisionServoController(kp_x=self.config.get('vision_servo', {}).get('control_gains', {}).get('Kp_x', 0.01))
        self.vision_servo = VisionServoController()
        self.grasp_predictor = GraspQualityPredictor()
        self.ik_planner = InverseKinematicsSolver()

        # 3. Parameter Management
        self.declare_parameter('debug_mode', True)
        self.debug_mode = self.get_parameter('debug_mode').value
        
        self.get_logger().info('Initializing Data-Driven Master Controller...')
        
        # 4. State Machine Initialization
        self.current_state = "IDLE"
        self.state_publisher = self.create_publisher(String, '/system/state', 10)
        
        # 5. Subscribing to the data streams
        self.create_subscription(Twist, '/cmd_vel', self.vision_cb, 10)
        self.create_subscription(Float32, '/grasp/quality_score', self.quality_cb, 10)
        self.create_subscription(Float32MultiArray, '/arm/joint_commands', self.ik_cb, 10)
        
        # 6. Tracking variables
        self.current_velocity = 1.0 
        self.grasp_score = 0.0
        self.trajectory_ready = False
        
        # 7. Orchestration Timer
        self.timer = self.create_timer(0.5, self.orchestration_callback)

    def vision_cb(self, msg):
        self.current_velocity = abs(msg.angular.z)

    def quality_cb(self, msg):
        self.grasp_score = msg.data

    def ik_cb(self, msg):
        if len(msg.data) == 6:
            self.trajectory_ready = True

    def orchestration_callback(self):
        previous_state = self.current_state

        if self.current_state == "IDLE":
            self.current_state = "SEARCHING"

        elif self.current_state == "SEARCHING":
            if self.current_velocity < 0.05:
                self.current_state = "EVALUATING"

        elif self.current_state == "EVALUATING":
            if self.grasp_score > 0.40:
                self.current_state = "PLANNING"
            else:
                self.current_state = "SEARCHING"

        elif self.current_state == "PLANNING":
            if self.trajectory_ready:
                self.current_state = "EXECUTING"

        elif self.current_state == "EXECUTING":
            self.get_logger().info("Grasp Successful!")
            self.current_state = "IDLE"
            self.current_velocity = 1.0 
            self.grasp_score = 0.0
            self.trajectory_ready = False

        # State transition logging and publishing
        if self.current_state != previous_state:
            self.get_logger().info(f"State Transition: {previous_state} -> {self.current_state}")
            msg = String()
            msg.data = self.current_state
            self.state_publisher.publish(msg)

        # Debug mode handling
        if self.debug_mode:
            # Only run heavy visualization/logging if debug_mode is True
            self.get_logger().debug(f"Visual Debugging Enabled | State: {self.current_state}")
            # Add specific debug logic here, e.g., drawing to a UI or publishing extra data