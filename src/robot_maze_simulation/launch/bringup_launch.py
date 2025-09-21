import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, TimerAction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
from launch.actions import OpaqueFunction
import subprocess

def generate_launch_description():
    simulation_pkg = get_package_share_directory('r' \
    'obot_maze_simulation')
    planners_pkg = get_package_share_directory('robot_maze_planners')
    description_pkg = get_package_share_directory('robot_maze_description')

    # Fixed world path (generated externally)
    world_candidates = [
        os.path.join(os.getcwd(), 'src', 'robot_maze_simulation', 'worlds', 'maze.world'),
        os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'worlds', 'maze.world')),
        os.path.join(simulation_pkg, 'worlds', 'maze.world'),
    ]
    world_file = next((p for p in world_candidates if os.path.exists(p)), world_candidates[0])

    if not os.path.exists(world_file):
        raise RuntimeError(
            "maze.world not found. Generate it first at one of these locations: "
            f"{world_candidates[0]} (preferred), {world_candidates[2]} (installed)."
        )

    # Robot URDF
    urdf_file = os.path.join(description_pkg, 'urdf', 'maze_generate', 'slambot.urdf.xacro')
    doc = xacro.process_file(urdf_file)
    robot_description = doc.toxml()

    # Args
    planner_arg = DeclareLaunchArgument('planner', default_value='astar', description='Planner type: astar | rrt | rrt_star')
    with_rviz_arg = DeclareLaunchArgument('with_rviz', default_value='true', description='Launch RViz2 alongside Gazebo')
    headless_arg = DeclareLaunchArgument('headless', default_value='false', description='Run server-only if true')
    sw_render_arg = DeclareLaunchArgument('force_software_rendering', default_value='true', description='Force llvmpipe for GUI')
    maze_rows_arg = DeclareLaunchArgument('maze_rows', default_value='10', description='Maze rows (cells)')
    maze_cols_arg = DeclareLaunchArgument('maze_cols', default_value='10', description='Maze cols (cells)')
    maze_compact_arg = DeclareLaunchArgument('maze_compact', default_value='true', description='Compact maze model')
    maze_cell_size_arg = DeclareLaunchArgument('cell_size', default_value='0.4', description='Maze cell size (m)')
    maze_seed_arg = DeclareLaunchArgument('maze_seed', default_value='', description='Optional seed')
    maze_force_regen_arg = DeclareLaunchArgument('maze_force_regen', default_value='false', description='No-op (compat)')
    minimal_gui_arg = DeclareLaunchArgument('minimal_gui', default_value='true', description='Minimal GUI plugins')
    maze_single_mesh_arg = DeclareLaunchArgument('maze_single_mesh', default_value='false', description='No-op (compat)')

    # LCs
    maze_rows = LaunchConfiguration('maze_rows')
    maze_cols = LaunchConfiguration('maze_cols')
    maze_cell_size = LaunchConfiguration('cell_size')
    planner = LaunchConfiguration('planner')
    force_sw = LaunchConfiguration('force_software_rendering')
    with_rviz = LaunchConfiguration('with_rviz')

    # Spawn and goal expressions (center the maze around origin)
    spawn_x_expr = PythonExpression(['- (', maze_cols, ' - 1) * ', maze_cell_size, ' / 2.0'])
    spawn_y_expr = PythonExpression(['- (', maze_rows, ' - 1) * ', maze_cell_size, ' / 2.0'])
    goal_x_expr = PythonExpression(['(', maze_cols, ' - 1) * ', maze_cell_size, ' / 2.0'])
    goal_y_expr = PythonExpression(['(', maze_rows, ' - 1) * ', maze_cell_size, ' / 2.0'])

    # Planner executable name
    planner_exec = PythonExpression(["'planner_' + '", planner, "'.replace('_','') + '_node'"])

    # Path to the map file
    map_yaml_file = os.path.join(simulation_pkg, 'maps', 'maze_map.yaml')

    # Optional static map (map_server) — resolve YAML and check package presence
    map_candidates = [
        os.path.join(os.getcwd(), 'src', 'robot_maze_simulation', 'maps', 'maze_map.yaml'),
        os.path.join(simulation_pkg, 'maps', 'maze_map.yaml'),
    ]
    map_yaml = next((p for p in map_candidates if os.path.exists(p)), map_candidates[0])
    have_map_yaml = os.path.exists(map_yaml)
    have_map_server_pkg = False
    try:
        r = subprocess.run(['ros2', 'pkg', 'prefix', 'nav2_map_server'], capture_output=True, text=True)
        have_map_server_pkg = (r.returncode == 0)
    except Exception:
        have_map_server_pkg = False

    # Check lifecycle manager availability
    have_lifecycle_pkg = False
    try:
        r = subprocess.run(['ros2', 'pkg', 'prefix', 'nav2_lifecycle_manager'], capture_output=True, text=True)
        have_lifecycle_pkg = (r.returncode == 0)
    except Exception:
        have_lifecycle_pkg = False

    map_server_actions = []
    if have_map_server_pkg and have_map_yaml:
        # Params file shipped with this package
        map_params_file = os.path.join(simulation_pkg, 'config', 'map_server_params.yaml')
        map_server_actions.append(
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[map_params_file, {'yaml_filename': map_yaml}],
            )
        )
        # Lifecycle manager to auto-configure / activate map_server
        if have_lifecycle_pkg:
            map_server_actions.append(
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_map',
                    output='screen',
                    parameters=[{'autostart': True, 'node_names': ['map_server']}],
                )
            )

    actions = [
        planner_arg, with_rviz_arg, headless_arg, sw_render_arg,
        maze_rows_arg, maze_cols_arg, maze_compact_arg, maze_single_mesh_arg,
        maze_cell_size_arg, maze_seed_arg, maze_force_regen_arg, minimal_gui_arg,

        # Minimal GUI config and VM-friendly GL env
        SetEnvironmentVariable('GZ_GUI_CONFIG_PATH', os.path.join(simulation_pkg, 'config', 'gui_minimal.config')),
        SetEnvironmentVariable('LIBGL_DRI3_DISABLE', '1', condition=IfCondition(PythonExpression(["'", LaunchConfiguration('headless'), "' == 'false' "]))),
        SetEnvironmentVariable('MESA_NO_ERROR', '1', condition=IfCondition(PythonExpression(["'", LaunchConfiguration('headless'), "' == 'false' "]))),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1', condition=IfCondition(PythonExpression(["'", LaunchConfiguration('headless'), "' == 'false' and '", force_sw, "' == 'true' "]))),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe', condition=IfCondition(PythonExpression(["'", LaunchConfiguration('headless'), "' == 'false' and '", force_sw, "' == 'true' "]))),
        SetEnvironmentVariable('QT_OPENGL', 'software', condition=IfCondition(PythonExpression(["'", LaunchConfiguration('headless'), "' == 'false' and '", force_sw, "' == 'true' "]))),
        SetEnvironmentVariable('QSG_RHI_BACKEND', 'software', condition=IfCondition(PythonExpression(["'", LaunchConfiguration('headless'), "' == 'false' and '", force_sw, "' == 'true' "]))),
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3', condition=IfCondition(PythonExpression(["'", LaunchConfiguration('headless'), "' == 'false' and '", force_sw, "' == 'true' "]))),
        SetEnvironmentVariable('MESA_GLSL_VERSION_OVERRIDE', '330', condition=IfCondition(PythonExpression(["'", LaunchConfiguration('headless'), "' == 'false' and '", force_sw, "' == 'true' "]))),

        # Gazebo GUI/server
        ExecuteProcess(
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('headless'), "' == 'false' "])) ,
            cmd=['gz', 'sim', '-v', '4', '-r', world_file],
            output='screen',
            name='gazebo_gui'
        ),
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    condition=IfCondition(PythonExpression(["'", LaunchConfiguration('headless'), "' == 'false' "])) ,
                    cmd=['gz', 'gui', '-c', PathJoinSubstitution([simulation_pkg, 'config', 'gui_minimal.config'])],
                    output='screen',
                    name='gazebo_gui_client'
                )
            ]
        ),
        ExecuteProcess(
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('headless'), "' == 'true' "])) ,
            cmd=['gz', 'sim', '-v', '4', '-r', '-s', world_file],
            output='screen',
            name='gazebo_server'
        ),

        # Spawn robot after world loads
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='ros_gz_sim', executable='create', output='screen',
                    arguments=['-topic', 'robot_description', '-name', 'slambot', '-x', spawn_x_expr, '-y', spawn_y_expr, '-z', '0.15']
                )
            ]
        ),

        Node(
            package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher', executable='joint_state_publisher', name='joint_state_publisher', output='screen'
        ),

        # Static TF: map -> odom (identity) so RViz and nodes can transform between frames
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='static_map_to_odom', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # Start planner (and optional map_server) after spawn settles
        TimerAction(
            period=8.0,
            actions=[
                GroupAction(
                    map_server_actions + [
                        Node(
                            package='robot_maze_planners', executable=planner_exec, name='maze_planner', output='screen',
                            parameters=[{
                                'goal_x': goal_x_expr, 'goal_y': goal_y_expr,
                                'default_start_x': PythonExpression(['- (', LaunchConfiguration('maze_cols'), ' - 1) * ', LaunchConfiguration('cell_size'), ' / 2.0']),
                                'default_start_y': PythonExpression(['- (', LaunchConfiguration('maze_rows'), ' - 1) * ', LaunchConfiguration('cell_size'), ' / 2.0']),
                                'allow_start_default': True,
                                'plan_on_timer': True,
                                'plan_rate_hz': 1.0,
                                'inflation_radius': 0.05,
                                'step_size': 0.2,
                                'max_iter': 500,
                                'radius': 0.5,
                                'resolution': 0.1
                            }]
                        )
                    ]
                )
            ]
        ),

        # Follower starts later
        TimerAction(
            period=11.0,
            actions=[
                Node(
                    package='robot_maze_planners', executable='path_follower_node', name='path_follower', output='screen',
                    parameters=[{
                        # Safer, more conservative defaults for tight maze corridors
                        'linear_gain': 0.6,
                        'max_linear_speed': 0.15,
                        'lookahead_distance': 0.3,
                        'ang_slowdown_threshold': 1.0,
                        'angular_gain': 3.5,
                        'max_angular_speed': 1.8,
                        'min_linear_speed': 0.08,
                        'recovery_forward_speed': 0.15,
                        'no_progress_timeout': 2.5,
                        'progress_min_delta': 0.05,
                    }]
                )
            ]
        ),

        # Odom->Pose republisher
        TimerAction(
            period=7.5,
            actions=[
                Node(package='robot_maze_planners', executable='odom_to_pose_node', name='odom_to_pose', output='screen')
            ]
        ),

        # Maze publisher (kept; planner can subscribe to this or /map)
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='robot_maze_planners', executable='maze_publisher_node', name='maze_publisher', output='screen',
                    parameters=[{'rows': LaunchConfiguration('maze_rows'), 'cols': LaunchConfiguration('maze_cols'), 'cell_size': LaunchConfiguration('cell_size')}]
                )
            ]
        ),

        # Bridges
        TimerAction(
            period=7.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                         '/model/slambot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                         '/model/slambot/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                         '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                         '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                         '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
                    output='screen', name='cmd_vel_bridge')
            ]
        ),

        # RViz
        TimerAction(
            period=9.0,
            actions=[
                Node(
                    condition=IfCondition(with_rviz), package='rviz2', executable='rviz2', name='rviz2',
                    arguments=['-d', PathJoinSubstitution([simulation_pkg, 'config', 'maze_nav_demo.rviz'])], output='screen'
                )
            ]
        ),
    ]

    return LaunchDescription(actions)
