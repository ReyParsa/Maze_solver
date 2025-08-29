import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    planner_arg = DeclareLaunchArgument(
        'planner', default_value='astar', description='Planner type: astar, rrt, rrt_star'
    )
    planner_map = {
        'astar': 'planner_astar_node',
        'rrt': 'planner_rrt_node',
        'rrt_star': 'planner_rrtstar_node',
    }
    planner = LaunchConfiguration('planner')
    return LaunchDescription([
        planner_arg,
        GroupAction([
            Node(
                package='robot_maze_planners',
                executable=planner,
                name='maze_planner',
                output='screen',
            )
        ])
    ])
