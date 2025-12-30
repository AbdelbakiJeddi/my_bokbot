#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class VelocityConverterNode(Node):
    def __init__(self):
        super().__init__('velocityconverter')
        self.publisher = self.create_publisher(TwistStamped, '/bumperbot_controller/cmd_vel', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
    
    def cmd_vel_callback(self, msg):
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.twist = msg
        self.publisher.publish(twist_stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityConverterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
