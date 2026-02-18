#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


def main(args=None):
    rclpy.init(args=args)
    node = Node("simple_path")
    pub = node.create_publisher(Twist, "cmd_vel", 10)
    
    distance = 1.0
    linear_vel = 1.0
    angular_vel = 1.0
    duration = distance / linear_vel
    
    msg = Twist()
    msg.linear.x = linear_vel
    msg.angular.z = 0.0
    
    start_time = time.time()
    while (time.time() - start_time) < duration:
        pub.publish(msg)
        time.sleep(0.05)
    
    stop_msg = Twist()
    stop_msg.linear.x = 0.0
    stop_msg.angular.z = 0.0
    pub.publish(stop_msg)
    
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
