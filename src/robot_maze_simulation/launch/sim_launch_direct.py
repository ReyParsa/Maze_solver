import os
import subprocess
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    simulation_pkg = get_package_share_directory('robot_maze_simulation')
    description_pkg = get_package_share_directory('robot_maze_description')
    world_file = os.path.join(simulation_pkg, 'worlds', 'maze_world.sdf')
    urdf_file = os.path.join(description_pkg, 'urdf', 'robot.urdf.xacro')
    
    # Process xacro file
    doc = xacro.process_file(urdf_file)
    robot_description = doc.toprettyxml(indent='  ')

    return LaunchDescription([
        # Launch Gazebo with direct gz command
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen',
            name='gazebo'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
    ])
