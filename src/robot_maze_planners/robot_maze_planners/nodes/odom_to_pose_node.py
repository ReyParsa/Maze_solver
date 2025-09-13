import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class OdomToPoseNode(Node):
    def __init__(self):
        # Use same base name as launch ("odom_to_pose") for clarity
        super().__init__('odom_to_pose')
        # Subscribe to the Gazebo model odometry topic (bridged to ROS)
        self._sub_model = self.create_subscription(
            Odometry, '/model/slambot/odom', self.odom_cb, 10)
        # Fallback generic odom (OdometryPublisher plugin publishes /odom)
        self._sub_generic = self.create_subscription(
            Odometry, 'odom', self.odom_cb, 10)
        # Publish a PoseStamped that planners expect on 'robot_pose'
        self.pose_pub = self.create_publisher(PoseStamped, 'robot_pose', 10)
        # internal flags
        self._got_first = False
        self._source_topic = None
        self.get_logger().info('Odom->Pose republisher started (listening on /model/slambot/odom and /odom)')
        # watchdog: warn if no odom after a few seconds
        self._watchdog_counter = 0
        self.create_timer(2.0, self._watchdog_check)

    def _watchdog_check(self):
        if not self._got_first:
            self._watchdog_counter += 1
            if self._watchdog_counter <= 5:  # limit warnings
                self.get_logger().warn('Still no /model/slambot/odom received. Check bridge and topic names.')

    def odom_cb(self, msg: Odometry):
        # Log a short summary and republish pose
        if not self._got_first:
            try:
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                # determine which topic delivered (heuristic: header.frame_id or pose)
                if self._source_topic is None:
                    # We can't directly know; inform user both subscriptions active
                    self._source_topic = 'unknown (either /model/slambot/odom or /odom)'
                self.get_logger().info(f'First odom received at ({x:.2f},{y:.2f}) from {self._source_topic}')
            except Exception:
                self.get_logger().info('First odom received')
            self._got_first = True
        else:
            # keep at debug after first
            try:
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                self.get_logger().debug(f'Received odom: ({x:.2f},{y:.2f})')
            except Exception:
                self.get_logger().debug('Received odom (unformatted)')

        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.pose_pub.publish(ps)
        # reduce noise: only debug per pose republish
        self.get_logger().debug('Published robot_pose')


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
