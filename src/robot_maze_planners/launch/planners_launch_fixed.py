import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    planner_arg = DeclareLaunchArgument(
        'planner', default_value='astar', description='Planner type: astar, rrt, rrt_star'
    )
    
    planner = LaunchConfiguration('planner')
    
    return LaunchDescription([
        planner_arg,
        
        # A* planner node
        Node(
            package='robot_maze_planners',
            executable='planner_astar_node',
            name='maze_planner',
            output='screen',
            condition=IfCondition(PythonExpression(["'", planner, "' == 'astar'"]))
        ),
        
        # RRT planner node
        Node(
            package='robot_maze_planners',
            executable='planner_rrt_node',
            name='maze_planner',
            output='screen',
            condition=IfCondition(PythonExpression(["'", planner, "' == 'rrt'"]))
        ),
        
        # RRT* planner node
        Node(
            package='robot_maze_planners',
            executable='planner_rrtstar_node',
            name='maze_planner',
            output='screen',
            condition=IfCondition(PythonExpression(["'", planner, "' == 'rrt_star'"]))
        ),
    ])
