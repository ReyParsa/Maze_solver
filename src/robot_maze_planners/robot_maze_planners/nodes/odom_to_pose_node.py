import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class OdomToPoseNode(Node):
    def __init__(self):
        super().__init__('odom_to_pose_node')
        self.create_subscription(Odometry, '/model/slambot/odom', self.odom_cb, 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'robot_pose', 10)
        self.get_logger().info('Odom->Pose republisher started')

    def odom_cb(self, msg: Odometry):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.pose_pub.publish(ps)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
