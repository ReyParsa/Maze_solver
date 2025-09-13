import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class MazePublisherNode(Node):
    def __init__(self):
        super().__init__('maze_publisher_node')
        self.pub = self.create_publisher(Path, 'maze_occupancy', 10)
        self.timer = self.create_timer(1.0, self.timer_cb)
        self.get_logger().info('Maze publisher started (publishing simple occupancy)')

    def timer_cb(self):
        # publish a trivial path encoding a few obstacle centers as poses (placeholder)
        msg = Path()
        msg.header.frame_id = 'map'
        for x,y in [(1.0,2.0),(2.0,2.0),(3.0,1.0)]:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = x
            p.pose.position.y = y
            msg.poses.append(p)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MazePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
