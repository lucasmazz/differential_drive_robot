import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Define the robot's name and package name
    robot_name = "differential_drive_robot"
    robot_pkg_name = "differential_drive_robot"

    # Set paths to Xacro model and configuration files
    robot_model_path = os.path.join(get_package_share_directory(
        robot_pkg_name), 'model', 'robot.xacro')
    
    gz_bridge_params_path = os.path.join(get_package_share_directory(
        robot_pkg_name), 'config', 'gz_bridge.yaml')

    # Process the Xacro file to generate the URDF representation of the robot
    robot_description = xacro.process_file(robot_model_path).toxml()

    # Define the path for a temporary URDF file to be used by Gazebo
    urdf_path = os.path.join('/tmp', 'robot.urdf')
    
    with open(urdf_path, 'w') as urdf_file:
        urdf_file.write(robot_description)

    # Prepare to include the Gazebo simulation launch file
    gazebo_pkg_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(
            'ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )

    # Include the Gazebo launch description with specific arguments
    gazebo_launch = IncludeLaunchDescription(
        gazebo_pkg_launch,
        launch_arguments={
            'gz_args': '-r -v 4 empty.sdf',   # Run with verbose output and an empty world
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Create a node to spawn the robot model in the Gazebo environment
    spawn_model_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,            # Name of the robot
            '-file', urdf_path,             # Path to the URDF file
            '-x', '0.0',                    # Initial X position
            '-y', '0.0',                    # Initial Y position
            '-z', '0.5',                    # Initial Z position
            '-R', '0.0',                    # Initial Roll
            '-P', '0.0',                    # Initial Pitch
            '-Y', '0.0',                    # Initial Yaw
            '-allow_renaming', 'false'
        ],
        output='screen'
    )

    # Create a node to publish the robot's state based on its URDF description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    # Create a node for the ROS-Gazebo bridge to handle message passing
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p',
                   f'config_file:={gz_bridge_params_path}'],
        output='screen'
    )

    # Create the LaunchDescription object and add all actions to it
    launch_description = LaunchDescription()
    launch_description.add_action(gazebo_launch)
    launch_description.add_action(spawn_model_gazebo_node)
    launch_description.add_action(robot_state_publisher_node)
    launch_description.add_action(gz_bridge_node)

    return launch_description
