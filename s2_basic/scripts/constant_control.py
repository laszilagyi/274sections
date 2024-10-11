#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import the message type to use
from std_msgs.msg import Int64, Bool, String
from geometry_msgs.msg import Twist

class ConstantControl(Node):
    def __init__(self) -> None:

        # initialize base class (must happen before everything else)
        super().__init__("constant_control")
        # a heartbeat counter
        self.cc_counter = 0
		# create publisher with: self.create_publisher(<msg type>, <topic>, <qos>)
        #self.cc_pub = self.create_publisher(String, "/constant_control", 10)
        self.cc_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # create a timer with: self.create_timer(<second>, <callback>)
        self.cc_timer = self.create_timer(1.0, self.cc_callback)
        
        # create subscription with: self.create_subscription(<msg type>, <topic>, <callback>, <qos>)
        self.cc_emergency = self.create_subscription(Bool, "/kill", self.cc_emergency_callback, 10)

    def cc_callback(self) -> None:

        # construct heartbeat message
        #msg = String()
        #msg.data = "sending constant control…"

        msg = Twist()  # 0 initialize everything by default
        msg.linear.x = 0.01  # set this to be the linear velocity
        msg.angular.z = 0.5 # set this to be the angular velocity

        # publish heartbeat counter
        self.cc_pub.publish(msg)
        # increment counter
        self.cc_counter += 1

    def cc_emergency_callback(self, msg: Bool) -> None:
        self.cc_timer.cancel()       
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cc_pub.publish(msg)
        print("Emergency stop.")

        
if __name__ == "__main__":
    rclpy.init()        # initialize ROS2 context (must run before any other rclpy call)
    node = ConstantControl()  # instantiate the heartbeat node
    rclpy.spin(node)    # Use ROS2 built-in schedular for executing the node
    rclpy.shutdown()    # cleanly shutdown ROS2 context