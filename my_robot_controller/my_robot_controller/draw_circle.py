#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    # DrawCircleNode node constructor
    def __init__(self):
        # Parent (Node) constructor
        super().__init__("draw_circle")
        # cmd_vel_publisher_ creation with data type, name and queue size
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        # timer function that repeats every half second the send_vel_command method
        self.timer = self.create_timer(0.5, self.send_vel_command)
        # Initialization message
        self.get_logger().info("Draw circle node has been started")

    # send_vel_command that executes the cmd_vel_publisher that publishes the message object
    def send_vel_command(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()