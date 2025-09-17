#!/usr/bin/env python3
"""One-shot publisher for /robot_description used by the bringup launch.

Reads robot_description from environment variable ROBOT_DESCRIPTION (set by the launch)
or from a file path given as the first argument. Publishes the full XML as std_msgs/String
once and exits.
"""
import sys
import os
import rclpy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node('publish_robot_description')

    pub = node.create_publisher(String, '/robot_description', 1)

    data = None
    # Prefer environment-provided payload
    data = os.environ.get('ROBOT_DESCRIPTION')
    if not data and len(sys.argv) > 1:
        path = sys.argv[1]
        with open(path, 'r') as f:
            data = f.read()

    if not data:
        node.get_logger().error('No robot description provided via ROBOT_DESCRIPTION or file arg')
        return 2

    msg = String()
    msg.data = data
    # publish once
    pub.publish(msg)
    node.get_logger().info('Published /robot_description (length=%d)' % len(msg.data))

    # give middleware a moment to send
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
