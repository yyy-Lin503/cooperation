#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class radarFollowerSilent(Node):
    def __init__(self):
        super().__init__('radar_follower')
        self.get_logger().info('控制决策通道并入一体化中心，本节点处于安全静默守护状态...')

def main(args=None):
    rclpy.init(args=args)
    node = radarFollowerSilent()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()