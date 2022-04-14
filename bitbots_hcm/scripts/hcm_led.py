#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from humanoid_league_msgs.msg import RobotControlState

BLINK_DURATION = 0.2
ERROR_TIMEOUT = 1
class HCMLedNode(Node):
    def __init__(self):
        rclpy.init(args=None)

        self.pub = self.create_publisher(ColorRGBA, "/led1", 1)

        self.last_state = -1

        sub = self.create_subscription(RobotControlState, "robot_state", self.hcm_state_cb, 1)
        rclpy.spin(self)


    def hcm_state_cb(msg: RobotControlState):
        state = msg.state
        # only do something if state changes, to not spam messages
        if state == self.last_state:
            return
        self.last_state = state
        led = ColorRGBA()
        led.a = 1.0
        if state == msg.CONTROLLABLE:
            led.r = 0.3
            led.g = 0.3
            led.b = 0.3
        elif state == msg.FALLING:
            led.r = 0.3
            led.g = 0.1
            led.b = 0
        elif state == msg.FALLEN:
            led.r = 0.3
            led.g = 0.3
            led.b = 0
        elif state == msg.HARDWARE_PROBLEM:
            led.r = 1
            led.g = 0
            led.b = 0
        elif state == msg.ANIMATION_RUNNING:
            led.r = 0
            led.g = 0
            led.b = 0.3
        elif state == msg.WALKING:
            led.r = 0
            led.g = 0.3
            led.b = 0
        elif state == msg.GETTING_UP:
            led.r = 0
            led.g = 0.3
            led.b = 0.3
        elif state == msg.HCM_OFF:
            led.r = 0
            led.g = 0
            led.b = 0
        elif state == msg.MOTOR_OFF:
            led.r = 0.03
            led.g = 0.03
            led.b = 0.03
        elif state == msg.KICKING:
            led.r = 0
            led.g = 0
            led.b = 0.1
        elif state == msg.PENALTY:
            led.r = 0.3
            led.g = 0
            led.b = 0.3
        elif state == msg.PENALTY_ANIMATION:
            led.r = 0.3
            led.g = 0
            led.b = 0.3
        elif state == msg.PICKED_UP:
            led.r = 0
            led.g = 0.03
            led.b = 0
        elif state == msg.RECORD:
            led.r = 0
            led.g = 0.1
            led.b = 0
        self.pub.publish(led)


