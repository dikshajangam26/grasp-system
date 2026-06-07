from setuptools import setup
import os
from glob import glob

package_name = 'grasp_system'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include your launch files here so the system can find them
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diksha',
    maintainer_email='diksha@todo.todo',
    description='Robot Grasping System',
    license='Apache-2.0',
    # THIS IS THE IMPORTANT PART:
    entry_points={
        'console_scripts': [
            'mock_camera = grasp_system.mock_camera:main',
            'vision_servoing = grasp_system.vision_servoing:main',
            'grasp_predictor = grasp_system.grasp_predictor:main',
            'ik_planner = grasp_system.ik_planner:main',
            'grasp_controller = grasp_system.grasp_controller:main',
        ],
    },
)