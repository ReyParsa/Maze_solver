import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    simulation_pkg = get_package_share_directory('robot_maze_simulation')
    planners_pkg = get_package_share_directory('robot_maze_planners')
    description_pkg = get_package_share_directory('robot_maze_description')
    world_file = os.path.join(simulation_pkg, 'worlds', 'maze_world.sdf')
    urdf_file = os.path.join(description_pkg, 'urdf', 'robot.urdf.xacro')
    
    # Process xacro file
    doc = xacro.process_file(urdf_file)
    robot_description = doc.toprettyxml(indent='  ')

    planner_arg = DeclareLaunchArgument(
        'planner', default_value='astar', description='Planner type: astar, rrt, rrt_star'
    )

    planner_node_map = {
        'astar': 'planner_astar_node.py',
        'rrt': 'planner_rrt_node.py',
        'rrt_star': 'planner_rrtstar_node.py',
    }

    planner = LaunchConfiguration('planner')

    return LaunchDescription([
        planner_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world_file}'}.items()
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
        GroupAction([
            Node(
                package='robot_maze_planners',
                executable=LaunchConfiguration('planner'),
                name='maze_planner',
                output='screen',
            )
        ])
    ])
