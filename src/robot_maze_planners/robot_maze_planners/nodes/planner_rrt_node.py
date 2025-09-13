import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from robot_maze_planners.planners.rrt_planner import RRTPlanner
from robot_maze_planners.utils.maze_helpers import parse_maze
from nav_msgs.msg import Odometry
import logging


class PlannerRRTNode(Node):
    def __init__(self):
        super().__init__('planner_rrt_node')
        # parameters (unified with A*)
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 2.0)
        self.declare_parameter('plan_on_timer', False)
        self.declare_parameter('plan_rate_hz', 1.0)
        self.declare_parameter('step_size', 0.2)
        self.declare_parameter('max_iter', 500)
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.goal = (
            self.get_parameter('goal_x').get_parameter_value().double_value,
            self.get_parameter('goal_y').get_parameter_value().double_value,
        )
        self.plan_on_timer = self.get_parameter('plan_on_timer').get_parameter_value().bool_value
        self.plan_rate_hz = self.get_parameter('plan_rate_hz').get_parameter_value().double_value
        self.step_size = self.get_parameter('step_size').get_parameter_value().double_value
        self.max_iter = self.get_parameter('max_iter').get_parameter_value().integer_value

        # publishers / subscribers
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.pose_sub = self.create_subscription(PoseStamped, 'robot_pose', self.pose_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/model/slambot/odom', self.odom_cb, 10)
        self.maze_sub = self.create_subscription(Path, 'maze_occupancy', self.maze_callback, 10)

        # state
        self.grid = None
        self.start = None
        self._last_logged_state = (None, None, None)  # (start, goal, grid_set)
        self._received_first_pose = False
        self._planning_timer = None
        if self.plan_on_timer:
            period = 1.0 / max(self.plan_rate_hz, 0.1)
            self._planning_timer = self.create_timer(period, self.try_plan)
        logging.basicConfig(level=logging.INFO)
        self.get_logger().info(f"RRT planner node started. goal={self.goal} step_size={self.step_size} max_iter={self.max_iter}")

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
        self.grid = parse_maze(msg)
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

        grid = self.grid if self.grid is not None else parse_maze(None)

        try:
            planner = RRTPlanner(grid, self.start, self.goal, self.step_size, self.max_iter)
            path_points = planner.plan()
        except Exception as exc:
            self.get_logger().error(f'RRT planning failed: {exc}')
            return

        if not path_points:
            self.get_logger().warn('RRT returned empty path.')
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
            self.get_logger().info(f'Published RRT path with {len(path_msg.poses)} poses (state change).')
        else:
            self.get_logger().debug(f'Republished RRT path ({len(path_msg.poses)} poses).')

    def set_goal(self, goal):
        self.goal = goal


def main(args=None):
    rclpy.init(args=args)
    node = PlannerRRTNode()
    if node.goal is None:
        node.set_goal((2.0, 2.0))
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
