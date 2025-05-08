from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpu6050',
            executable='mpu_node',
            name='mpu6050_node',
            parameters=[os.path.join(
                os.path.dirname(__file__), '../config/mpu6050_params.yaml')],
            output='screen'
        )
    ])
