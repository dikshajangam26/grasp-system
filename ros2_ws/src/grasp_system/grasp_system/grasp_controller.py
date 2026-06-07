#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from grasp_system.vision_servoing import VisionServoController
from grasp_system.grasp_predictor import GraspQualityPredictor
from grasp_system.ik_planner import InverseKinematicsSolver

class GraspSystemController(Node):
    def __init__(self):
        super().__init__('grasp_system_controller')
        
        self.get_logger().info('Initializing Data-Driven Master Controller...')
        
        self.current_state = "IDLE"
        self.state_publisher = self.create_publisher(String, '/system/state', 10)
        
        # Subscribing to the data streams of our sub-modules
        self.create_subscription(Twist, '/cmd_vel', self.vision_cb, 10)
        self.create_subscription(Float32, '/grasp/quality_score', self.quality_cb, 10)
        self.create_subscription(Float32MultiArray, '/arm/joint_commands', self.ik_cb, 10)
        
        # Data tracking variables for Task 5.3
        self.current_velocity = 1.0  # Start high so it doesn't instantly trigger
        self.grasp_score = 0.0
        self.trajectory_ready = False
        
        self.timer = self.create_timer(0.5, self.orchestration_callback)

    def vision_cb(self, msg):
        """Task 5.3: Camera frame -> Vision Servoing -> Velocity stabilized"""
        # Monitor the rotational velocity. If it drops near 0, the target is centered.
        self.current_velocity = abs(msg.angular.z)

    def quality_cb(self, msg):
        """Task 5.3: Grasp predictor output"""
        self.grasp_score = msg.data

    def ik_cb(self, msg):
        """Task 5.3: Joint angles + trajectory output"""
        if len(msg.data) == 6:
            self.trajectory_ready = True

    def orchestration_callback(self):
        previous_state = self.current_state

        if self.current_state == "IDLE":
            self.current_state = "SEARCHING"

        elif self.current_state == "SEARCHING":
            # Data-Driven Check: Has the velocity stabilized below 0.05 rad/s?
            if self.current_velocity < 0.05:
                self.get_logger().info(f"[Message Passing] Velocity stabilized at {self.current_velocity:.3f}. Triggering Predictor.")
                self.current_state = "EVALUATING"
            else:
                self.get_logger().debug(f"[Message Passing] Still tracking. Current Vel: {self.current_velocity:.2f}")

        elif self.current_state == "EVALUATING":
            # Data-Driven Check: Is the quality score > threshold?
            if self.grasp_score > 0.40:
                self.get_logger().info(f"[Message Passing] Valid grasp found (Score: {self.grasp_score:.2f}). Triggering IK Planner.")
                self.current_state = "PLANNING"
            else:
                self.get_logger().warning(f"Grasp score ({self.grasp_score:.2f}) too low! Returning to SEARCHING...")
                self.current_state = "SEARCHING"

        elif self.current_state == "PLANNING":
            # Data-Driven Check: Has the IK planner published a valid set of joint commands?
            if self.trajectory_ready:
                self.get_logger().info("[Message Passing] 6-DOF Trajectory received. Publishing to Arm Controller.")
                self.current_state = "EXECUTING"

        elif self.current_state == "EXECUTING":
            # In a real setup, we would wait for a 'done' message from the hardware motors.
            # We will clear the flags and reset to IDLE for the next loop.
            self.get_logger().info("Grasp Successful! Object secured.")
            self.current_state = "IDLE"
            
            # Reset data flags for the next cycle
            self.current_velocity = 1.0 
            self.grasp_score = 0.0
            self.trajectory_ready = False

        if self.current_state != previous_state:
            self.get_logger().info(f"[Master State Transition]: {previous_state} ---> {self.current_state}")

        msg = String()
        msg.data = self.current_state
        self.state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    vision_node = VisionServoController()
    predictor_node = GraspQualityPredictor()
    ik_node = InverseKinematicsSolver()
    master_node = GraspSystemController()
    
    executor = MultiThreadedExecutor()
    executor.add_node(vision_node)
    executor.add_node(predictor_node)
    executor.add_node(ik_node)
    executor.add_node(master_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        vision_node.destroy_node()
        predictor_node.destroy_node()
        ik_node.destroy_node()
        master_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()