#!/usr/bin/env python3
"""
POLEBOT Web Dashboard Server
Serves real-time robot status via WebSocket (rosbridge compatible)
TODO: Implement full web dashboard in next sprint
"""
import rclpy
from rclpy.node import Node

class DashboardServer(Node):
    def __init__(self):
        super().__init__('dashboard_server')
        self.get_logger().info('Dashboard server started (stub - implementation pending)')

def main(args=None):
    rclpy.init(args=args)
    node = DashboardServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
