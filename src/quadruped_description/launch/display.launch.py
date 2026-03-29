import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'quadruped_description'
    pkg_share = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'go1.urdf')
    
    # 1. Define the path to your saved RViz config
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'display.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # 2. Tell RViz to load the config file on startup
            arguments=['-d', rviz_config_file] 
        )
    ])