import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math


class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')
        self.path = []
        self.current_idx = 0
        self.pose = None
        self.declare_parameter('linear_gain', 0.8)
        self.declare_parameter('angular_gain', 2.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.linear_gain = self.get_parameter('linear_gain').get_parameter_value().double_value
        self.angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value

    # subscriptions and publishers
    self.path_sub = self.create_subscription(Path, 'planned_path', self.path_cb, 10)
    self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
    # publish to the Gazebo model-specific cmd_vel topic so the bridge/Gazebo plugin receives it
    self.cmd_pub = self.create_publisher(Twist, '/model/slambot/cmd_vel', 10)

    # timer for control loop
    self.timer = self.create_timer(0.1, self.timer_cb)
    self.get_logger().info('Path follower started')

    def path_cb(self, msg: Path):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.current_idx = 0
        self.get_logger().info(f'Received planned path with {len(self.path)} points')

    def odom_cb(self, msg: Odometry):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, self._yaw_from_quat(msg.pose.pose.orientation))

    def _yaw_from_quat(self, q):
        # quaternion to yaw
        x, y, z, w = q.x, q.y, q.z, q.w
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny, cosy)

    def timer_cb(self):
        if not self.path or self.pose is None:
            return
        # clamp current index
        if self.current_idx >= len(self.path):
            # reached end
            self.cmd_pub.publish(Twist())
            return

        tx, ty = self.path[self.current_idx]
        px, py, yaw = self.pose
        dx = tx - px
        dy = ty - py
        dist = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        ang_error = self._angle_diff(angle_to_target, yaw)

        # if close to this waypoint, advance
        if dist < self.goal_tolerance:
            self.current_idx += 1
            return

        # compute control
        twist = Twist()
        # slow down if angular error large
        if abs(ang_error) > 0.4:
            twist.linear.x = 0.0
            twist.angular.z = max(-1.0, min(1.0, self.angular_gain * ang_error))
        else:
            twist.linear.x = max(-0.0, min(0.6, self.linear_gain * dist))
            twist.angular.z = max(-1.0, min(1.0, self.angular_gain * ang_error))

        self.cmd_pub.publish(twist)

    def _angle_diff(self, a, b):
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
