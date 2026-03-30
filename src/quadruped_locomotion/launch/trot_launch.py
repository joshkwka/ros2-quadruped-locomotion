from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='quadruped_locomotion',
            executable='trot_node',
            name='trot_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])