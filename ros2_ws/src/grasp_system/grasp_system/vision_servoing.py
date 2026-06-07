#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from cv_bridge import CvBridge

class VisionServoController(Node):
    def __init__(self):
        super().__init__('vision_servo_controller')
        
        self.image_subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.error_history_x = []
        self.error_history_y = []
        self.history_length = 5
        
        # --- TASK 2.3 ADDITIONS: Control Parameters ---
        # Proportional gains for X and Y axes
        self.Kp_x = 0.01
        self.Kp_y = 0.01
        
        # Velocity saturation limits (m/s or rad/s depending on the mapped axis)
        self.max_velocity_x = 0.5 
        self.max_velocity_y = 0.5 
        
        self.get_logger().info('Vision Servo Controller initialized. Waiting for images...')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        if hasattr(cv2.aruco, 'ArucoDetector'):
            # Newer OpenCV 4.7+ syntax (Used by GitHub Actions/Pip)
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, rejected = detector.detectMarkers(cv_image)
        else:
            # Older OpenCV 4.5 syntax (Used by Ubuntu 22.04/Apt)
            corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)        height, width, _ = cv_image.shape
        
        desired_x = width / 2.0
        desired_y = height / 2.0
        
        # Initialize a Twist message. By default, all velocities are 0.0
        cmd_msg = Twist()
        
        if ids is not None:
            marker_corners = corners[0][0] 
            current_x = np.mean(marker_corners[:, 0])
            current_y = np.mean(marker_corners[:, 1])
            
            raw_error_x = desired_x - current_x
            raw_error_y = desired_y - current_y
            
            self.error_history_x.append(raw_error_x)
            self.error_history_y.append(raw_error_y)
            
            if len(self.error_history_x) > self.history_length:
                self.error_history_x.pop(0)
                self.error_history_y.pop(0)
                
            filtered_error_x = np.median(self.error_history_x)
            filtered_error_y = np.median(self.error_history_y)
            
            # --- TASK 2.3 ADDITIONS: Control Law ---
            
            # 1. Compute proportional velocity commands
            v_x = self.Kp_x * filtered_error_x
            v_y = self.Kp_y * filtered_error_y
            
            # 2. Add velocity saturation (Clamp values between -max and +max)
            v_x_clamped = max(-self.max_velocity_x, min(self.max_velocity_x, v_x))
            v_y_clamped = max(-self.max_velocity_y, min(self.max_velocity_y, v_y))
            
            # 3. Map to standard Twist kinematics
            # Note: X error in an image usually dictates left/right turning (angular z)
            # Y error in an image usually dictates up/down or forward/back (linear z or x)
            cmd_msg.angular.z = v_x_clamped  
            cmd_msg.linear.z = v_y_clamped   
            
            self.get_logger().info(f"Tracking | Error X: {filtered_error_x:.1f}, Cmd Z: {cmd_msg.angular.z:.2f}")
            
        else:
            # 4. Handle edge cases (marker lost, out of bounds)
            # If ids is None, we don't compute velocities. cmd_msg remains all 0.0.
            self.get_logger().debug("Marker lost or out of bounds. Stopping movement.")
        
        # Publish the computed velocity. If tracking, it moves. If lost, it actively stops.
        self.velocity_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    vision_servo_controller = VisionServoController()
    
    try:
        rclpy.spin(vision_servo_controller)
    except KeyboardInterrupt:
        pass
    finally:
        vision_servo_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()