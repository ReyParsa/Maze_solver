import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class OdomToPoseNode(Node):
    def __init__(self):
        # Use same base name as launch ("odom_to_pose") for clarity
        super().__init__('odom_to_pose')
        # Parameter: prefer model odom when both are present
        self.declare_parameter('prefer_model', True)
        self._prefer_model = self.get_parameter('prefer_model').get_parameter_value().bool_value

        # Subscribe to the Gazebo model odometry topic (bridged to ROS)
        self._sub_model = self.create_subscription(
            Odometry, '/model/slambot/odom', self._odom_cb_model, 10)
        # Fallback generic odom (OdometryPublisher plugin publishes /odom)
        self._sub_generic = self.create_subscription(
            Odometry, '/odom', self._odom_cb_generic, 10)
        # Publish a PoseStamped that planners expect on 'robot_pose'
        self.pose_pub = self.create_publisher(PoseStamped, 'robot_pose', 10)
        # internal flags
        self._got_first = False
        self._source_topic = None  # one of: 'model', 'generic'
        self.get_logger().info('Odom->Pose republisher started (listening on /model/slambot/odom and /odom)')
        # watchdog: warn if no odom after a few seconds
        self._watchdog_counter = 0
        self.create_timer(2.0, self._watchdog_check)

    def _watchdog_check(self):
        if not self._got_first:
            self._watchdog_counter += 1
            if self._watchdog_counter <= 5:  # limit warnings
                self.get_logger().warn('Still no /model/slambot/odom received. Check bridge and topic names.')

    def _odom_cb_model(self, msg: Odometry):
        # If we prefer model, lock to it upon first reception
        if self._source_topic is None:
            self._source_topic = 'model'
            self.get_logger().info('Locking robot_pose source to /model/slambot/odom (preferred)')
        if self._source_topic != 'model':
            # Already locked to generic; ignore model updates
            return
        self._publish_pose(msg, source='model')

    def _odom_cb_generic(self, msg: Odometry):
        # If model isn't preferred or hasn't arrived yet, allow generic
        if self._source_topic is None:
            if not self._prefer_model:
                self._source_topic = 'generic'
                self.get_logger().info('Locking robot_pose source to /odom (prefer_model=false)')
            else:
                # prefer model, but if model hasn't arrived for a while, fall back after first generic
                self._source_topic = 'generic'
                self.get_logger().info('Temporarily using /odom until /model/slambot/odom arrives')
        elif self._source_topic == 'model':
            # already locked to model; ignore generic updates
            return
        self._publish_pose(msg, source='generic')

    def _publish_pose(self, msg: Odometry, source: str):
        # Log a short summary and republish pose
        if not self._got_first:
            try:
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                self.get_logger().info(f'First odom received at ({x:.2f},{y:.2f}) from {source}')
            except Exception:
                self.get_logger().info(f'First odom received from {source}')
            self._got_first = True
        else:
            # keep at debug after first
            try:
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                self.get_logger().debug(f'Received odom[{source}]: ({x:.2f},{y:.2f})')
            except Exception:
                self.get_logger().debug(f'Received odom[{source}] (unformatted)')

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
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        # Avoid RCLError on double shutdown
        try:
            rclpy.shutdown()
        except Exception:
            pass
