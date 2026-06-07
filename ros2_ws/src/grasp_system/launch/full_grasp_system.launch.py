from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments for easy command-line overrides
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo/Webots) clock if true'),

        # 1. Camera / Image Source Node
        Node(
            package='grasp_system',
            executable='mock_camera',
            name='camera_source',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_rate': 30.0,
                'resolution_width': 640,
                'resolution_height': 480
            }]
        ),

        # 2. Vision Servoing Node
        Node(
            package='grasp_system',
            executable='vision_servoing',
            name='vision_servoing',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'kp_x': 1.5,           # Proportional gain X
                'kp_y': 1.5,           # Proportional gain Y
                'kd_x': 0.1,           # Derivative gain X
                'kd_y': 0.1,           # Derivative gain Y
                'error_tolerance': 0.02 # Alignment threshold in meters
            }]
        ),

        # 3. Grasp Predictor Node
        Node(
            package='grasp_system',
            executable='grasp_predictor',
            name='grasp_predictor',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'confidence_threshold': 0.85,
                # Mapping to the models volume inside your container
                'model_path': '/ros2_ws/models/grasp_quality_model.pth' 
            }]
        ),

        # 4. Inverse Kinematics (IK) Planner Node
        Node(
            package='grasp_system',
            executable='ik_planner',
            name='ik_planner',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'max_iterations': 200,
                'ik_tolerance': 1e-3,
                'planning_group': 'manipulator'
            }]
        ),

        # 5. Main Controller Node
        Node(
            package='grasp_system',
            executable='grasp_controller',
            name='main_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'state_publish_rate': 50.0,
                'safety_z_clearance': 0.15 # Safe approach height in meters
            }]
        )
    ])