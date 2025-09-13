import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from robot_maze_planners.planners.astar_planner import AStarPlanner
from robot_maze_planners.utils.maze_helpers import parse_maze
import logging


class PlannerAStarNode(Node):
    def __init__(self):
        """Initialize the A* planner node.

        This re-written method ensures consistent 4-space indentation (avoids mixed tabs/spaces)
        and cleanly declares/reads parameters before setting up publishers/subscribers/state.
        """
        super().__init__('planner_astar_node')

        # ---- Parameters ----
        param_defaults = {
            'resolution': 0.1,  # retained for backward compat (alias of cell_size)
            'cell_size': 0.4,
            'goal_x': 2.0,
            'goal_y': 2.0,
            'plan_on_timer': False,
            'plan_rate_hz': 1.0,
            'allow_start_default': True,
            'default_start_x': 0.0,
            'default_start_y': 0.0,
        }
        for name, value in param_defaults.items():
            self.declare_parameter(name, value)

        # read back (correct indentation)
        gp = self.get_parameter
        self.resolution = gp('resolution').get_parameter_value().double_value
        self.cell_size = gp('cell_size').get_parameter_value().double_value or self.resolution
        self.goal = (
            gp('goal_x').get_parameter_value().double_value,
            gp('goal_y').get_parameter_value().double_value,
        )
        self.plan_on_timer = gp('plan_on_timer').get_parameter_value().bool_value
        self.plan_rate_hz = gp('plan_rate_hz').get_parameter_value().double_value
        self.allow_start_default = gp('allow_start_default').get_parameter_value().bool_value
        self.default_start = (
            gp('default_start_x').get_parameter_value().double_value,
            gp('default_start_y').get_parameter_value().double_value,
        )

        # ---- Interfaces ----
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.pose_sub = self.create_subscription(PoseStamped, 'robot_pose', self.pose_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/model/slambot/odom', self.odom_cb, 10)
        self.maze_sub = self.create_subscription(Path, 'maze_occupancy', self.maze_callback, 10)

        # ---- State ----
        self.grid = None
        self.start = None
        self._last_logged_state = (None, None, None)  # (start, goal, grid_set)
        self._received_first_pose = False
        self._planning_timer = None
        if self.plan_on_timer:
            period = 1.0 / max(self.plan_rate_hz, 0.1)
            self._planning_timer = self.create_timer(period, self.try_plan)

        # watchdog to warn if start never arrives (and optionally set default start)
        self._start_watchdog = self.create_timer(2.0, self._check_start)

        self.get_logger().info(
            f"A* planner node started. goal={self.goal} cell_size={self.cell_size} plan_on_timer={self.plan_on_timer}"
        )
        self.get_logger().info(
            f"Params: allow_start_default={self.allow_start_default} default_start={self.default_start} plan_rate_hz={self.plan_rate_hz}"
        )

    def _check_start(self):
        if self.start is None:
            if self.allow_start_default:
                self.start = self.default_start
                self.get_logger().warn(f'No start pose received yet; using default start {self.start}')
                self.try_plan()
            else:
                self.get_logger().warn('No start pose received yet (still waiting for /robot_pose or /model/slambot/odom).')
        else:
            # once we have start, remove the watchdog timer
            if self._start_watchdog is not None:
                self._start_watchdog.cancel()
                self._start_watchdog = None
    def pose_callback(self, msg):
        self.start = (msg.pose.position.x, msg.pose.position.y)
        if not self._received_first_pose:
            self.get_logger().info(f'Received first robot_pose: {self.start}')
            self._received_first_pose = True
        elif self._last_logged_state[0] != self.start:
            self.get_logger().info(f'Received robot pose: {self.start}')
            self._last_logged_state = (self.start, self._last_logged_state[1], self._last_logged_state[2])
        self.try_plan()

    def odom_cb(self, msg: Odometry):
        self.start = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self._last_logged_state[0] != self.start:
            self.get_logger().info(f'Received odom pose: {self.start}')
            self._last_logged_state = (self.start, self._last_logged_state[1], self._last_logged_state[2])
        self.try_plan()

    def maze_callback(self, msg):
        # Convert incoming wall segments path to occupancy grid with current cell_size.
        self.grid = parse_maze(msg, cell_size=self.cell_size)
        if self._last_logged_state[2] != (self.grid is not None):
            self.get_logger().info('Received maze occupancy.')
            self._last_logged_state = (self._last_logged_state[0], self._last_logged_state[1], (self.grid is not None))
        self.try_plan()

    def try_plan(self):
        # Evaluate if a replan is needed (state change or periodic timer)
        state = (self.start, self.goal, self.grid is not None)
        state_changed = state != self._last_logged_state
        if not state_changed and not self.plan_on_timer:
            return
        self._last_logged_state = state

        if not self.start or not self.goal:
            return
        self.get_logger().debug(f'Planning attempt: start={self.start} goal={self.goal} grid_ready={self.grid is not None}')
        if self.grid is not None:
            grid = self.grid
        else:
            # still waiting for maze publication; use temporary empty grid just to allow motion
            grid = parse_maze(None, rows=25, cols=25, cell_size=self.cell_size)

        try:
            planner = AStarPlanner(grid, self.start, self.goal, cell_size=self.cell_size)
            path_points = planner.plan()
        except Exception as exc:
            self.get_logger().error(f'Planning failed: {exc}')
            return

        if not path_points:
            self.get_logger().warn('Planner returned empty path.')
            return

        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for x, y in path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        if state_changed:
            self.get_logger().info(f'Published A* path with {len(path_msg.poses)} poses (state change).')
        else:
            self.get_logger().debug(f'Republished A* path ({len(path_msg.poses)} poses).')

    def set_goal(self, goal):
        self.goal = goal


def main(args=None):
    rclpy.init(args=args)
    node = PlannerAStarNode()
    # goal can be set via parameters; retain manual override for compatibility
    if node.goal is None:
        node.set_goal((2.0, 2.0))
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
