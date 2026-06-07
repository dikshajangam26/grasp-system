#!/usr/bin/env python3
import rclpy
import cv2
from grasp_system.grasp_predictor import GraspQualityPredictor

def main():
    rclpy.init()
    node = GraspQualityPredictor()
    
    # Load a sample image (ensure this file exists!)
    sample_img = cv2.imread('examples/data/sample_grasp.jpg')
    if sample_img is None:
        node.get_logger().warn("Sample image not found, using blank frame.")
        sample_img = cv2.imread('tests/data/mock_image.png') # or fallback
        
    score, candidates = node.predict_grasp_quality(sample_img)
    node.get_logger().info(f"Grasp Prediction Score: {score:.4f}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()