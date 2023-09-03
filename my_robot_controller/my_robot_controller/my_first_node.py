#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    # MyFirstNode class constructor
    def __init__(self):
        # Parent class (Node) constructor
        super().__init__("my_first_node")
        # Create_Timer executes timer_callback every 1 sec
        self.create_timer(1.0, self.timer_callback)

    # Timer_callback that prints "Hello"
    def timer_callback(self):
        self.get_logger().info("Hello")
    

def main(args=None):
    rclpy.init(args=args) # ROS2 communications initialization
    node = MyFirstNode() # Node instance creation
    rclpy.spin(node) # Keeps node alive and running
    rclpy.shutdown() # ROS2 communications shutdown

if __name__ == '__main__':
    main()