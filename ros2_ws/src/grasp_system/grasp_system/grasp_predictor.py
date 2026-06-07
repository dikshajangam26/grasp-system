#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
import torch
import torchvision.models as models
import torch.nn as nn
from torchvision import transforms
import os
from grasp_system.performance_metrics import PerformanceMonitor

class GraspQualityPredictor(Node):
    def __init__(self):
        super().__init__('grasp_quality_predictor')
        
        self.bridge = CvBridge()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model_path = 'models/grasp_quality_model.pth'
        self.model = None
        
        # Initialize Performance Monitor
        self.perf_monitor = PerformanceMonitor()
        
        self.preprocess = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])
        
        self.load_model()
        
        if self.model is not None:
            self.image_sub = self.create_subscription(
                Image, '/camera/image_raw', self.image_callback, 10
            )
            self.quality_pub = self.create_publisher(Float32, '/grasp/quality_score', 10)
            self.pose_pub = self.create_publisher(Pose, '/grasp/best_pose', 10)
            
            self.get_logger().info('Grasp Quality Predictor node is fully active and listening.')

    def load_model(self):
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"CRITICAL: Model file not found at {self.model_path}!")
            return
        try:
            weights = models.MobileNet_V2_Weights.DEFAULT
            self.model = models.mobilenet_v2(weights=weights)
            self.model.classifier[1] = nn.Sequential(
                nn.Linear(self.model.last_channel, 1),
                nn.Sigmoid()
            )
            self.model.load_state_dict(torch.load(self.model_path, map_location=self.device, weights_only=True))
            self.model.to(self.device)
            self.model.eval()
            self.get_logger().info('Neural Network weights loaded successfully.')
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            self.model = None

    def predict_grasp_quality(self, cv_image):
        try:
            # Measure Preprocessing Time
            with self.perf_monitor.time_block('grasp_preprocessing'):
                rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                input_tensor = self.preprocess(rgb_image)
            
            # Measure Inference Time
            with self.perf_monitor.time_block('grasp_inference'):
                input_batch = input_tensor.unsqueeze(0).to(self.device)
                with torch.no_grad():
                    output = self.model(input_batch)
                    probability = output.item()
            
            candidates = [
                {"x": 320.0, "y": 240.0, "angle": 0.0, "score": probability},
                {"x": 310.0, "y": 230.0, "angle": 45.0, "score": probability * 0.9},
                {"x": 330.0, "y": 250.0, "angle": -45.0, "score": probability * 0.85}
            ]
            return probability, candidates
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            return 0.0, []

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        score, candidates = self.predict_grasp_quality(cv_image)
        
        if not candidates:
            return

        best_candidate = max(candidates, key=lambda x: x['score'])
        
        score_msg = Float32()
        score_msg.data = best_candidate['score']
        self.quality_pub.publish(score_msg)
        
        pose_msg = Pose()
        pose_msg.position.x = float(best_candidate['x']) / 1000.0
        pose_msg.position.y = float(best_candidate['y']) / 1000.0
        pose_msg.position.z = 0.5
        
        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f"Published Grasp | Score: {score_msg.data:.2f}")

def main(args=None):
    rclpy.init(args=args)
    grasp_predictor = GraspQualityPredictor()
    try:
        rclpy.spin(grasp_predictor)
    except KeyboardInterrupt:
        pass
    finally:
        grasp_predictor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()