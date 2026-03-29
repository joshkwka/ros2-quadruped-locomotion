import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch.actions import AppendEnvironmentVariable

def generate_launch_description():
    # Paths to files
    pkg_description = get_package_share_directory('quadruped_description')
    urdf_file = os.path.join(pkg_description, 'urdf', 'go1.urdf')
    
    # Tell Gazebo where to find the meshes
    workspace_share_dir = os.path.dirname(pkg_description)
    set_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=workspace_share_dir
    )
    
    # Let Xacro evaluate the $(find ...) tags into absolute paths
    robot_description = {'robot_description': Command(['xacro ', urdf_file])}

    # Robot State Publisher Node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}] 
    )

    # Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(), 
    )

    # Spawn the Robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'go1', 
            '-allow_renaming', 'true',
            '-z', '0.4'
        ],
        output='screen',
    )

    # Controller Spawners (Turn on YAML configs)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
    )

    leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_controller", "--controller-manager", "controller_manager"],
    )

    delayed_joint_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delayed_leg_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[leg_controller_spawner],
        )
    )

    # Bridge the Simulation Clock to ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        set_resource_path,
        clock_bridge,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        delayed_joint_broadcaster,
        delayed_leg_controller,
    ])