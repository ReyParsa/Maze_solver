from setuptools import setup

package_name = 'robot_maze_planners'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/planners_launch.py', 'launch/planners_launch_fixed.py']),
        ('share/' + package_name + '/resource', ['resource/robot_maze_planners']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Maze Team',
    maintainer_email='user@example.com',
    description='Path planning algorithms (A*, RRT, RRT*) for robot maze solver',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_astar_node = robot_maze_planners.nodes.planner_astar_node:main',
            'planner_rrt_node = robot_maze_planners.nodes.planner_rrt_node:main',
            'planner_rrtstar_node = robot_maze_planners.nodes.planner_rrtstar_node:main',
        ],
    },
)
