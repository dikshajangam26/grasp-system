#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Float32MultiArray
from geometry_msgs.msg import Twist

# Importing the nodes directly from the package
from grasp_system.vision_servoing import VisionServoController
from grasp_system.grasp_predictor import GraspQualityPredictor
from grasp_system.ik_planner import InverseKinematicsSolver

class GraspSystemController(Node):
    def __init__(self):
        super().__init__('grasp_system_controller')
        
        # Initialize sub-modules as attributes to match the benchmark script
        self.vision_servo = VisionServoController()
        self.grasp_predictor = GraspQualityPredictor()
        self.ik_planner = InverseKinematicsSolver()

        self.declare_parameter('debug_mode', True)
        self.debug_mode = self.get_parameter('debug_mode').value
        
        self.get_logger().info('Initializing Data-Driven Master Controller...')
        
        self.current_state = "IDLE"
        self.state_publisher = self.create_publisher(String, '/system/state', 10)
        
        # Subscribing to the data streams
        self.create_subscription(Twist, '/cmd_vel', self.vision_cb, 10)
        self.create_subscription(Float32, '/grasp/quality_score', self.quality_cb, 10)
        self.create_subscription(Float32MultiArray, '/arm/joint_commands', self.ik_cb, 10)
        
        # Tracking variables
        self.current_velocity = 1.0 
        self.grasp_score = 0.0
        self.trajectory_ready = False
        
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

        if self.current_state != previous_state:
            self.get_logger().info(f"State Transition: {previous_state} -> {self.current_state}")
            msg = String()
            msg.data = self.current_state
            self.state_publisher.publish(msg)

        if self.debug_mode:
            # Only run heavy visualization/logging if debug_mode is True
            self.get_logger().info(f"Visual Debugging Enabled | State: {self.current_state}")
            # Add specific debug logic here, e.g., drawing to a UI or publishing extra data