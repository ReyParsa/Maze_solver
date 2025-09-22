#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid

class MapQoSBridge(Node):
    def __init__(self):
        super().__init__('map_qos_bridge')
        
        # QoS for subscribing to map_server (TRANSIENT_LOCAL)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # QoS for publishing to RViz (VOLATILE)
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            sub_qos
        )
        
        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/map_rviz',
            pub_qos
        )
        
        self.get_logger().info('Map QoS bridge started - republishing /map to /map_rviz')
    
    def map_callback(self, msg):
        # Republish the map with volatile QoS
        self.publisher.publish(msg)
        self.get_logger().info(f'Republished map: {msg.info.width}x{msg.info.height}')

def main(args=None):
    rclpy.init(args=args)
    bridge = MapQoSBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()