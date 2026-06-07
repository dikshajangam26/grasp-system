import matplotlib.pyplot as plt
import numpy as np

def plot_performance_report(controller):
    # Retrieve data
    vision_data = controller.vision_servo.perf_monitor.get_stats('vision_detection')
    grasp_data = controller.grasp_predictor.perf_monitor.get_stats('grasp_inference')
    ik_data = controller.ik_planner.perf_monitor.get_stats('ik_solving')

    # Filter out the first iteration (the "warm-up" spike)
    # We slice from [1:] to skip index 0
    vision_list = vision_data['all'][1:]
    grasp_list = grasp_data['all'][1:]
    ik_list = ik_data['all'][1:]

    # Create figure
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    datasets = [vision_list, grasp_list, ik_list]
    titles = ['Vision Detection', 'Grasp Inference', 'IK Solving']
    colors = ['blue', 'green', 'red']
    # Set rough X-limits based on your current observed data
    x_limits = [(0, 5), (0, 30), (7, 13)] 

    for i, ax in enumerate(axes):
        ax.hist(datasets[i], bins=15, color=colors[i], edgecolor='black')
        ax.set_title(titles[i])
        ax.set_xlabel('Latency (ms)')
        ax.set_ylabel('Frequency')
        ax.set_xlim(x_limits[i]) # Force balanced scaling

    plt.tight_layout()
    plt.suptitle("System Performance (Steady State)", y=1.02, fontsize=14)
    plt.show()