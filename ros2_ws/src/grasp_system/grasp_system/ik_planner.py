#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
import ikpy.chain
from ikpy.link import OriginLink, URDFLink
import numpy as np

class InverseKinematicsSolver(Node):
    def __init__(self):
        super().__init__('ik_solver')
        
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
        
        # --- TASK 4.4 ADDITIONS: Track current state ---
        # Assume the robot starts completely straight up at "home" (0 radians for all joints)
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

    def solve_ik(self, target_position):
        # --- NEW SAFETY CHECK: Handle invalid inputs instantly ---
        if target_position is None:
            self.get_logger().error("Invalid input: target_position is None.")
            return None

        # Now it is safe to do math
        distance = np.linalg.norm(target_position)
        if distance > 1.1:
            return None

        try:
            raw_angles = self.arm_chain.inverse_kinematics(
                target_position=target_position,
                target_orientation=[0, 0, -1],
                orientation_mode="Z"
            )

            fk_position = self.arm_chain.forward_kinematics(raw_angles)[:3, 3]
            error = np.linalg.norm(fk_position - target_position)

            if error > 0.05:
                return None

            joint_angles = raw_angles[1:]
            
            if not self.validate_constraints(joint_angles):
                return None
                
            return joint_angles
            
        except Exception as e:
            self.get_logger().error(f"IK computation failed: {e}")
            return None
            
    def plan_trajectory(self, target_angles):
        """Task 4.4: Generate waypoints and validate the entire path."""
        num_waypoints = 10
        
        # Use NumPy to mathematically slice the distance between 'current' and 'target' into 10 even steps
        waypoints = np.linspace(self.current_joint_angles, target_angles, num=num_waypoints)
        
        valid_trajectory = []
        for i, waypoint in enumerate(waypoints):
            # Validate every single micro-step along the way
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
            # Check if the robot is already at the target to prevent spamming trajectory calculations
            if np.allclose(self.current_joint_angles, target_angles, atol=0.01):
                return
                
            # --- TASK 4.4 ADDITIONS: Plan path before publishing ---
            trajectory = self.plan_trajectory(target_angles)
            
            if trajectory is not None:
                self.get_logger().info(f"Planned safe trajectory with {len(trajectory)} waypoints.")
                
                # In a full simulation, we would publish the waypoints one by one on a timer.
                # For this integration test, we publish the final target and update our internal state.
                msg_out = Float32MultiArray()
                msg_out.data = [float(a) for a in target_angles]
                self.joint_publisher.publish(msg_out)
                
                # Update the robot's current position to the new target
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