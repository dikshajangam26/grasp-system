#!/usr/bin/env python3
"""
Benchmark the full autonomous grasp system.
Usage: 
    source /opt/ros/humble/setup.bash
    export PYTHONPATH=$PWD:$PWD/ros2_ws/src/grasp_system:$PYTHONPATH
    python3 examples/benchmark_system.py
"""

import rclpy
import time
from grasp_system.grasp_controller import GraspSystemController
from tests.conftest import create_mock_image
from sensor_msgs.msg import Image # Add this import
from cv_bridge import CvBridge # Add this import
from examples.plot_performance import plot_performance_report

def benchmark_full_system(num_iterations=100):
    # Initialize ROS 2
    if not rclpy.ok():
        rclpy.init()
        
    controller = GraspSystemController()
    bridge = CvBridge() # Initialize bridge
    
    print("=" * 60)
    print("AUTONOMOUS GRASP SYSTEM - PERFORMANCE BENCHMARK")
    print("=" * 60)
    print(f"Running {num_iterations} iterations...\n")
    
    for i in range(num_iterations):

        raw_image = create_mock_image()
        
        # Convert numpy array to ROS Image message
        ros_image = bridge.cv2_to_imgmsg(raw_image, "bgr8")
        
        # Now pass the proper ROS message to your nodes
        controller.vision_servo.image_callback(ros_image)
        controller.grasp_predictor.image_callback(ros_image)
        
        # For IK, you are passing a list, keep it as is
        target_pose = [0.3, 0.2, 0.5, 0, 0, 0]
        controller.ik_planner.solve_ik(target_pose)
        
        time.sleep(0.01)  # Minimal sleep to allow CPU context switching

    plot_performance_report(controller)    

    # Print report
    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)
    
    # Access performance stats using the correct attribute names
    # Note: Ensure the string keys inside get_stats match the names used in your time_block() calls
    vision_latency = controller.vision_servo.perf_monitor.get_stats('vision_detection')
    grasp_latency = controller.grasp_predictor.perf_monitor.get_stats('grasp_inference')
    ik_latency = controller.ik_planner.perf_monitor.get_stats('ik_solving')
    
    print(f"\nVision Servoing:")
    print(f"  Mean:    {vision_latency['mean']:.2f} ms")
    print(f"  Min:     {vision_latency['min']:.2f} ms")
    print(f"  Max:     {vision_latency['max']:.2f} ms")
    print(f"  FPS:     {1000 / vision_latency['mean']:.1f}")
    
    print(f"\nGrasp Prediction:")
    print(f"  Mean:    {grasp_latency['mean']:.2f} ms")
    print(f"  Min:     {grasp_latency['min']:.2f} ms")
    print(f"  Max:     {grasp_latency['max']:.2f} ms")
    
    print(f"\nIK Planning:")
    print(f"  Mean:    {ik_latency['mean']:.2f} ms")
    print(f"  Min:     {ik_latency['min']:.2f} ms")
    print(f"  Max:     {ik_latency['max']:.2f} ms")
    
    print("\n" + "=" * 60)
    print("DEPLOYMENT READINESS")
    print("=" * 60)
    
    if vision_latency['mean'] < 33:
        print("✅ Vision Servoing: Real-time capable (>30 FPS)")
    else:
        print("⚠️  Vision Servoing: Below real-time (optimization required)")
    
    if grasp_latency['mean'] < 100:
        print("✅ Grasp Prediction: Edge-deployment ready (<100ms)")
    else:
        print("⚠️  Grasp Prediction: Needs optimization")
    
    if ik_latency['mean'] < 50:
        print("✅ IK Planning: Fast (<50ms)")
    else:
        print("⚠️  IK Planning: Consider faster solver")

    # Cleanup
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    benchmark_full_system()