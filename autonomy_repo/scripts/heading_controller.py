#!/usr/bin/env python3

import numpy as np
import rclpy
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState

class HeadingController(BaseHeadingController):
    def __init__(self):
        super().__init__()
        #self.kp = 2.0
        self.declare_parameter("kp", 2.0)

    @property
    def kp(self) -> float:
        return self.get_parameter("kp").value

    def compute_control_with_goal(self, current_state: TurtleBotState, goal_state: TurtleBotState) -> TurtleBotControl:
        heading_error = wrap_angle(goal_state.theta - current_state.theta)
        omega = self.kp * heading_error

        control_message = TurtleBotControl(v=0.0, omega=omega)
        
        return control_message    

if __name__ == "__main__":
    rclpy.init()
    heading_controller = HeadingController()
    rclpy.spin(heading_controller)
    rclpy.shutdown()