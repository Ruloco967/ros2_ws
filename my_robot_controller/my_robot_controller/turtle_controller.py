#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from functools import partial

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.previous_x = 0
        self.cmd_vel_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback,10)
        self.get_logger().info("Turtle controller has been started.")
    
    def pose_callback(self, pose: Pose):
        cmd = Twist()
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 1.5
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        # call_set_pen_service execution inside the callback for pose
        if pose.x > 5.5 and self.previous_x <= 5.5:
            self.previous_x = pose.x
            self.get_logger().info("Set color to red!")
            self.call_set_pen_service(255, 0, 0, 3, 0)
        elif pose.x <= 5.5 and self.previous_x > 5.5:
            self.previous_x = pose.x
            self.get_logger().info("Set color to green!")
            self.call_set_pen_service(0, 255, 0, 3, 0)
    
    def call_set_pen_service(self, r, g, b, width, off):
        # Client that calls the service "/turtle1/set_pen" of data type SetPen
        client = self.create_client(SetPen, "/turtle1/set_pen")
        # Make sure the service is available. If there is no service available, the message is sent
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")
        # When it is available, a request is created
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        # A future object is created to send the request to the server
        future = client.call_async(request)
        # A callback is added
        future.add_done_callback(partial(self.callback_set_pen))
    
    # When the service replies, the callback function below is executed
    def callback_set_pen(self, future):
        try:
            # response is created and gets the result from the future object generated
            response = future.result()
        except Exception as e:
            # error handlings
            self.get_logger().error("Service call failed: %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()