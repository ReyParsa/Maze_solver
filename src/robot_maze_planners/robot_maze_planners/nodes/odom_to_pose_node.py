import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class OdomToPoseNode(Node):
    def __init__(self):
        super().__init__('odom_to_pose_node')
        # Subscribe to the Gazebo model odometry topic (bridged to ROS)
        self.create_subscription(Odometry, '/model/slambot/odom', self.odom_cb, 10)
        # Publish a PoseStamped that planners expect on 'robot_pose'
        self.pose_pub = self.create_publisher(PoseStamped, 'robot_pose', 10)
        self.get_logger().info('Odom->Pose republisher started')

    def odom_cb(self, msg: Odometry):
        # Log a short summary and republish pose
        try:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.get_logger().debug(f'Received odom: pos=({x:.2f},{y:.2f})')
        except Exception:
            self.get_logger().debug('Received odom (unable to format)')

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
