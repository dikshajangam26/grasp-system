#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
import ikpy.chain
from ikpy.link import OriginLink, URDFLink
import numpy as np
from grasp_system.performance_metrics import PerformanceMonitor

class InverseKinematicsSolver(Node):
    def __init__(self):
        super().__init__('ik_solver')
        
        # Initialize Performance Monitor
        self.perf_monitor = PerformanceMonitor()
        
        self.arm_chain = ikpy.chain.Chain(name='6_dof_mock_arm', links=[
            OriginLink(),
            URDFLink(name="shoulder_pan", origin_translation=[0, 0, 0.1], origin_orientation=[0, 0, 0], rotation=[0, 0, 1]),
            URDFLink(name="shoulder_lift", origin_translation=[0, 0, 0.1], origin_orientation=[0, 1.57, 0], rotation=[0, 1, 0]),
            URDFLink(name="elbow", origin_translation=[0.4, 0, 0], origin_orientation=[0, 0, 0], rotation=[0, 1, 0]),
            URDFLink(name="wrist_1", origin_translation=[0.4, 0, 0], origin_orientation=[0, 0, 0], rotation=[0, 1, 0]),
            URDFLink(name="wrist_2", origin_translation=[0, 0, 0.1], origin_orientation=[0, 0, 1.57], rotation=[0, 0, 1]),
            URDFLink(name="wrist_3", origin_translation=[0, 0, 0.1], origin_orientation=[0, 0, 0], rotation=[0, 1, 0])
        ], active_links_mask=[False, True, True, True, True, True, True])
        
        self.joint_limits = [
            (-2.0 * np.pi, 2.0 * np.pi),  
            (-2.0 * np.pi, 2.0 * np.pi),  
            (-np.pi, np.pi),              
            (-2.0 * np.pi, 2.0 * np.pi),  
            (-2.0 * np.pi, 2.0 * np.pi),  
            (-2.0 * np.pi, 2.0 * np.pi)   
        ]
        
        self.current_joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.pose_subscriber = self.create_subscription(Pose, '/grasp/best_pose', self.pose_callback, 10)
        self.joint_publisher = self.create_publisher(Float32MultiArray, '/arm/joint_commands', 10)
        self.get_logger().info('IK Solver initialized and listening for grasp targets.')

    def validate_constraints(self, joint_angles):
        for i, angle in enumerate(joint_angles):
            q_min, q_max = self.joint_limits[i]
            if not (q_min <= angle <= q_max):
                return False
        
        elbow_angle = joint_angles[2]
        shoulder_lift = joint_angles[1]
        if abs(elbow_angle) > 2.5 and abs(shoulder_lift) > 2.5:
            return False
            
        return True

    def solve_ik(self, target_pose_input):

        if target_pose_input is None:
            return None
        """
        Solves Inverse Kinematics for a given target pose.
        target_pose_input: Can be a 3-element [x, y, z] or 6-element [x, y, z, roll, pitch, yaw] list/array.
        """
        # 1. Correctly slice the input to ensure we only have [x, y, z]
        target_position = target_pose_input[:3]
        
        # 2. Check if the sliced input is valid
        if target_position is None:
            self.get_logger().error("Invalid input: target_position is None.")
            return None

        # 3. Check workspace reachability
        distance = np.linalg.norm(target_position)
        if distance > 1.1:
            return None

        # 4. Measure IK Solving Time
        try:
            with self.perf_monitor.time_block('ik_solving'):
                raw_angles = self.arm_chain.inverse_kinematics(
                    target_position=target_position,
                    target_orientation=[0, 0, -1],
                    orientation_mode="Z"
                )

                # Validate with Forward Kinematics
                fk_position = self.arm_chain.forward_kinematics(raw_angles)[:3, 3]
                error = np.linalg.norm(fk_position - target_position)

                if error > 0.05:
                    return None

                # Exclude the fixed base link
                joint_angles = raw_angles[1:]
                
                # Check hardware limits
                if not self.validate_constraints(joint_angles):
                    return None
                    
                return joint_angles
            
        except Exception as e:
            self.get_logger().error(f"IK computation failed: {e}")
            return None
            
    def plan_trajectory(self, target_angles):
        """Generate waypoints and validate the entire path."""
        num_waypoints = 10
        
        # Measure Trajectory Planning Time
        with self.perf_monitor.time_block('trajectory_planning'):
            waypoints = np.linspace(self.current_joint_angles, target_angles, num=num_waypoints)
            
            valid_trajectory = []
            for i, waypoint in enumerate(waypoints):
                if self.validate_constraints(waypoint):
                    valid_trajectory.append(waypoint)
                else:
                    self.get_logger().error(f"Trajectory blocked! Waypoint {i+1} hits a physical constraint.")
                    return None
                    
            return valid_trajectory

    def pose_callback(self, msg):
        target_position = [msg.position.x, msg.position.y, msg.position.z]
        target_angles = self.solve_ik(target_position)
        
        if target_angles is not None:
            if np.allclose(self.current_joint_angles, target_angles, atol=0.01):
                return
                
            trajectory = self.plan_trajectory(target_angles)
            
            if trajectory is not None:
                self.get_logger().info(f"Planned safe trajectory with {len(trajectory)} waypoints.")
                
                msg_out = Float32MultiArray()
                msg_out.data = [float(a) for a in target_angles]
                self.joint_publisher.publish(msg_out)
                
                self.current_joint_angles = target_angles
                
                angles_deg = [round(np.degrees(a), 1) for a in target_angles]
                self.get_logger().info(f"Execution complete. Final Angles (deg): {angles_deg}")

def main(args=None):
    rclpy.init(args=args)
    ik_solver = InverseKinematicsSolver()
    try:
        rclpy.spin(ik_solver)
    except KeyboardInterrupt:
        pass
    finally:
        ik_solver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()