import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from robot_maze_planners.planners.rrt_planner import RRTPlanner
from robot_maze_planners.utils.maze_helpers import parse_maze
import logging

class PlannerRRTNode(Node):
    def __init__(self):
        super().__init__('planner_rrt_node')
        self.declare_parameter('step_size', 0.2)
        self.declare_parameter('max_iter', 500)
        self.step_size = self.get_parameter('step_size').get_parameter_value().double_value
        self.max_iter = self.get_parameter('max_iter').get_parameter_value().integer_value
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.pose_sub = self.create_subscription(PoseStamped, 'robot_pose', self.pose_callback, 10)
        self.maze_sub = self.create_subscription(Path, 'maze_occupancy', self.maze_callback, 10)
        self.grid = None
        self.start = None
        self.goal = None
        logging.basicConfig(level=logging.INFO)
        self.get_logger().info('RRT planner node started.')

    def pose_callback(self, msg):
        self.start = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'Received robot pose: {self.start}')
        self.try_plan()

    def maze_callback(self, msg):
        self.grid = parse_maze(msg)
        self.get_logger().info('Received maze occupancy.')
        self.try_plan()

    def try_plan(self):
        if self.start and self.goal:
            grid = self.grid
            if grid is None:
                from robot_maze_planners.utils.maze_helpers import parse_maze
                grid = parse_maze(None)
            planner = RRTPlanner(grid, self.start, self.goal, self.step_size, self.max_iter)
            path_points = planner.plan()
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
            self.get_logger().info('Published RRT path.')

    def set_goal(self, goal):
        self.goal = goal


def main(args=None):
    rclpy.init(args=args)
    node = PlannerRRTNode()
    node.set_goal((2.0, 2.0))  # Example goal
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
