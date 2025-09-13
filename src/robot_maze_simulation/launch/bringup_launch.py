import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, TimerAction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
from launch.actions import OpaqueFunction
import subprocess

def generate_launch_description():
    simulation_pkg = get_package_share_directory('robot_maze_simulation')
    planners_pkg = get_package_share_directory('robot_maze_planners')
    description_pkg = get_package_share_directory('robot_maze_description')
    # use fixed maze world (pre-generated) to avoid MazeGenerate plugin dependency
    world_file = os.path.join(simulation_pkg, 'worlds', 'maze_world_fixed.sdf')
    # use slambot from MazeGenerate integration
    urdf_file = os.path.join(description_pkg, 'urdf', 'maze_generate', 'slambot.urdf.xacro')
    
    # Process xacro file
    doc = xacro.process_file(urdf_file)
    robot_description = doc.toxml()  # compact robot_description

    planner_arg = DeclareLaunchArgument(
        'planner', default_value='astar', description='Planner type: astar | rrt | rrt_star'
    )

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false', description='Run gz sim headless (server-only) if true'
    )

    maze_rows_arg = DeclareLaunchArgument(
        'maze_rows', default_value='10', description='Maze rows (cells)')
    maze_cols_arg = DeclareLaunchArgument(
        'maze_cols', default_value='10', description='Maze cols (cells)')
    maze_compact_arg = DeclareLaunchArgument(
        'maze_compact', default_value='true', description='Use compact mode for maze (single model)')

    maze_cell_size_arg = DeclareLaunchArgument(
        'cell_size', default_value='0.4', description='Size of a maze cell in meters')
    maze_seed_arg = DeclareLaunchArgument(
        'maze_seed', default_value='', description='Optional integer seed for deterministic maze generation')
    maze_force_regen_arg = DeclareLaunchArgument(
        'maze_force_regen', default_value='false', description='If true, force regeneration even when cached seeded world exists')

    maze_rows = LaunchConfiguration('maze_rows')
    maze_cols = LaunchConfiguration('maze_cols')
    maze_compact = LaunchConfiguration('maze_compact')
    maze_single_mesh_arg = DeclareLaunchArgument(
        'maze_single_mesh', default_value='false', description='Export single mesh STL and reference it in the SDF')
    maze_single_mesh = LaunchConfiguration('maze_single_mesh')
    maze_cell_size = LaunchConfiguration('cell_size')
    maze_seed = LaunchConfiguration('maze_seed')
    maze_force_regen = LaunchConfiguration('maze_force_regen')

    planner = LaunchConfiguration('planner')

    # Expressions for centered maze spawn (maze generated around origin)
    # Lower-left (start) corner coordinates (negative half-extent)
    spawn_x_expr = PythonExpression(['- (', maze_cols, ' - 1) * ', maze_cell_size, ' / 2.0'])
    spawn_y_expr = PythonExpression(['- (', maze_rows, ' - 1) * ', maze_cell_size, ' / 2.0'])
    # Upper-right (goal) corner coordinates (positive half-extent)
    goal_x_expr = PythonExpression(['(', maze_cols, ' - 1) * ', maze_cell_size, ' / 2.0'])
    goal_y_expr = PythonExpression(['(', maze_rows, ' - 1) * ', maze_cell_size, ' / 2.0'])

    # compute planner executable name: planner_<name_without_underscores>_node
    planner_exec = PythonExpression(["'planner_' + '", planner, "'.replace('_','') + '_node'"])

    # generate a random maze SDF each run and prefer it
    generated_world = '/tmp/maze_world_generated.sdf'
    # Prefer workspace src path (development), then source-relative path, then installed share
    workspace_src_candidate = os.path.join(os.getcwd(), 'src', 'robot_maze_simulation', 'tools', 'generate_maze_sdf.py')
    generator_py_src = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'tools', 'generate_maze_sdf.py'))
    generator_py_installed = os.path.join(simulation_pkg, 'tools', 'generate_maze_sdf.py')
    if os.path.exists(workspace_src_candidate):
        generator_py = workspace_src_candidate
    elif os.path.exists(generator_py_src):
        generator_py = generator_py_src
    elif os.path.exists(generator_py_installed):
        generator_py = generator_py_installed
    else:
        generator_py = workspace_src_candidate  # fallback for error message
    print('Generator candidates: workspace_src=%s, src=%s, installed=%s' % (workspace_src_candidate, generator_py_src, generator_py_installed))
    print('Selected generator:', generator_py)

    # world launch configuration (will be set at runtime by generator action)
    world_arg = DeclareLaunchArgument('world', default_value=world_file)

    def _run_generator_and_set_world(context, *args, **kwargs):
        # context-aware runtime generator function executed via OpaqueFunction
        try:
            rows_val = int(context.perform_substitution(maze_rows))
            cols_val = int(context.perform_substitution(maze_cols))
        except Exception:
            rows_val = 10
            cols_val = 10
        compact_flag = context.perform_substitution(maze_compact).lower() in ('true', '1', 'yes')
        single_mesh_flag = context.perform_substitution(maze_single_mesh).lower() in ('true','1','yes')
        try:
            cell_size_val = float(context.perform_substitution(maze_cell_size))
        except Exception:
            cell_size_val = 0.4
        seed_raw = context.perform_substitution(maze_seed)
        seed_val = None
        if seed_raw and seed_raw.strip() not in ('', 'None', 'none'):
            try:
                seed_val = int(seed_raw)
            except Exception:
                seed_val = None

        force_regen_flag = context.perform_substitution(maze_force_regen).lower() in ('true', '1', 'yes')

        # compute generated world path
        generated_world_local = generated_world
        if seed_val is not None:
            cs_str = f"{cell_size_val:.2f}".replace('.', 'p')
            mesh_suffix = '_mesh' if single_mesh_flag else ''
            cache_dir = os.path.join(simulation_pkg, 'generated')
            os.makedirs(cache_dir, exist_ok=True)
            generated_world_local = os.path.join(cache_dir, f'maze_seed{seed_val}_r{rows_val}_c{cols_val}_cs{cs_str}{mesh_suffix}.sdf')

        # decide whether to run generator
        if seed_val is not None and os.path.exists(generated_world_local) and not force_regen_flag:
            print('Using cached generated world at', generated_world_local)
        else:
            if not os.path.exists(generator_py):
                print('Maze generator script not found at', generator_py)
            else:
                cmd = ['python3', generator_py, '--rows', str(rows_val), '--cols', str(cols_val), '--cell_size', str(cell_size_val), '--output', generated_world_local]
                if seed_val is not None:
                    cmd.extend(['--seed', str(seed_val)])
                if compact_flag:
                    cmd.append('--compact')
                if single_mesh_flag:
                    cmd.append('--single-mesh')
                print('Running maze generator (using):', generator_py)
                print('Command:', ' '.join(cmd))
                try:
                    subprocess.run(cmd, check=True)
                except subprocess.CalledProcessError as e:
                    print('Maze generator failed:', e)

        # return action to set the world launch configuration to the generated path
        return [SetLaunchConfiguration(name='world', value=generated_world_local)]

    # runtime generator action (resolves launch substitutions correctly)
    generator_action = OpaqueFunction(function=_run_generator_and_set_world)

    world_to_run = generated_world if os.path.exists(generated_world) else world_file
    # debug info about selected world
    try:
        exists = os.path.exists(world_to_run)
        size = os.path.getsize(world_to_run) if exists else 0
        wall_count = 0
        if exists:
            with open(world_to_run, 'r') as wf:
                data = wf.read()
            wall_count = data.count("<model name='wall_") + data.count('<model name="wall_')
        print(f"World to run: {world_to_run} (exists={exists}, size={size} bytes, wall_models={wall_count})")
    except Exception as e:
        print('Failed to stat world file:', e)

    return LaunchDescription([
        planner_arg,
        headless_arg,
        maze_rows_arg,
        maze_cols_arg,
        maze_compact_arg,
        maze_single_mesh_arg,
        maze_cell_size_arg,
        maze_seed_arg,
        maze_force_regen_arg,
        world_arg,
        generator_action,
        # start gz directly to avoid launch include issues with paths containing spaces
        # start gz sim with verbose logging (-v 4). use server-only (-s) when headless==true
        # GUI (default)
        ExecuteProcess(
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('headless'), "' == 'false'"])),
            cmd=['gz', 'sim', '-v', '4', '-r', LaunchConfiguration('world')],
            output='screen',
            name='gazebo_gui'
        ),
        # start the GUI client (separate process) when not headless to ensure a window appears
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    condition=IfCondition(PythonExpression(["'", LaunchConfiguration('headless'), "' == 'false'"])),
                    cmd=['gz', 'gui'],
                    output='screen',
                    name='gazebo_gui_client'
                )
            ]
        ),
        # server-only (headless)
        ExecuteProcess(
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('headless'), "' == 'true'"])),
            cmd=['gz', 'sim', '-v', '4', '-r', '-s', LaunchConfiguration('world')],
            output='screen',
            name='gazebo_server'
        ),
        # delay spawn slightly so gz sim finishes loading the world
        TimerAction(
            # give the server + GUI a bit more time to initialize before spawning the robot
            period=6.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=['-topic', 'robot_description', '-name', 'slambot',
                               '-x', spawn_x_expr,
                               '-y', spawn_y_expr,
                               '-z', '0.15'],
                    output='screen',
                )
            ]
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
        # Delay planner startup to allow robot spawn and physics to settle
        TimerAction(
            period=8.0,
            actions=[
                GroupAction([
                    Node(
                        package='robot_maze_planners',
                        executable=planner_exec,
                        name='maze_planner',
                        output='screen',
                        parameters=[
                            {
                                # Use explicit numeric goals (user requested 9.6,9.6) but fall back to centered corner if smaller maze
                                'goal_x': 9.6,
                                'goal_y': 9.6,
                                'default_start_x': 0.0,
                                'default_start_y': 0.0,
                                'allow_start_default': True,
                                'plan_on_timer': True,
                                'plan_rate_hz': 1.0,
                                # algorithm-specific (unused by A* if irrelevant)
                                'step_size': 0.2,
                                'max_iter': 500,
                                'radius': 0.5,
                                'resolution': 0.1
                            }
                        ],
                    )
                ])
            ]
        )
        ,
        # Start path follower after planner has had time to publish a planned_path
        TimerAction(
            period=11.0,
            actions=[
                Node(
                    package='robot_maze_planners',
                    executable='path_follower_node',
                    name='path_follower',
                    output='screen',
                )
            ]
        )
        ,
        # Start odom->pose republisher after spawn so planners get PoseStamped on 'robot_pose'
        TimerAction(
            period=7.5,
            actions=[
                Node(
                    package='robot_maze_planners',
                    executable='odom_to_pose_node',
                    name='odom_to_pose',
                    output='screen',
                )
            ]
        ),
        # publish a simple maze occupancy so planners can start
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='robot_maze_planners',
                    executable='maze_publisher_node',
                    name='maze_publisher',
                    output='screen',
                    parameters=[
                        {'rows': LaunchConfiguration('maze_rows'),
                         'cols': LaunchConfiguration('maze_cols'),
                         'cell_size': LaunchConfiguration('cell_size')}
                    ]
                )
            ]
        ),
        # Start a ros_gz_bridge parameter_bridge for the Gazebo model cmd_vel topic so Gazebo subscribes to ROS messages
        TimerAction(
            period=7.0,
            actions=[
                ExecuteProcess(
                cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    # model-scoped topics
                    '/model/slambot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/slambot/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    # generic robot-level topics (DiffDrive plugin publishes cmd_vel only, OdometryPublisher publishes /odom)
                    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    # optional pose info (if pose publisher plugin added later)
                    '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
                    output='screen',
                    name='cmd_vel_bridge'
                )
            ]
        )
    ])
