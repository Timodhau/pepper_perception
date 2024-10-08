#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Walk - Small example to make Nao walk"""

import qi
import argparse
import sys
import time
import random
import rospy
from std_msgs.msg import String, Int64


class MoveHead():
    def __init__(self, parser_):

        # Naoqi connection
        self.parser = parser_
        self.parser.add_argument("--ip", type=str, default="169.254.112.44",
                                 help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
        self.parser.add_argument("--port", type=int, default=9559,
                                 help="Naoqi port number")
        self.args = self.parser.parse_args()
        self.session = qi.Session()
        try:
            self.session.connect("tcp://" + self.args.ip + ":" + str(self.args.port))
        except RuntimeError:
            print("Can't connect to Naoqi at ip \"" + self.args.ip + "\" on port " + str(
                self.args.port) + ".\n" + "Please check your script arguments. Run with -h option for help.")
            sys.exit(1)
        self.motion_service = self.session.service("ALMotion")

        # Publishers
        self.pub_is_moving = rospy.Publisher('/internal_state/is_moving', Int64, queue_size=10)
        # Subscribers
        self.head_move_sub = rospy.Subscriber("/action/head_move", String, self.callback_head_move)
        self.motion_service.moveInit()

    # ROS Callback
    def callback_head_move(self, movement_ros):
        move_type = movement_ros.data
        self.pub_is_moving.publish(1)
        if move_type in ["right", "left"]:
            self.check_side(move_type)
        if move_type == "front":
            self.check_front()
        self.pub_is_moving.publish(0)

    def check_side(self, side):
        if side == "left":
            self.motion_service.setAngles("HeadYaw", 0.8, 0.1)
        if side == "right":
            self.motion_service.setAngles("HeadYaw", -0.8, 0.1)
        time.sleep(1)

    def check_front(self):
        self.motion_service.setAngles("HeadYaw", 0, 0.1)
        time.sleep(1)

def main(session):
    """
    Walk: Small example to make Nao walk with jerky head.
    This example is only compatible with NAO.
    """
    # Get the services ALMotion & ALRobotPosture.

    motion_service = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")

    # Wake up robot
    # motion_service.wakeUp()
    motion_service.moveInit()

    # Send robot to Stand Init
    posture_service.goToPosture("StandInit", 0.2)
    motion_service.setAngles("HeadPitch", -0.45, 0.1)
    time.sleep(2)
    # Initialize the move process.
    # Check the robot pose and take a right posture.
    # This is blocking called.
    motion_service.moveInit()
    # JERKY HEAD
    motion_service.setAngles("HeadYaw", -0.8, 0.1)
    time.sleep(2)
    motion_service.setAngles("HeadYaw", 0., 0.1)
    time.sleep(2)
    motion_service.setAngles("HeadPitch", -0.45, 0.1)
    # stop move on the next double support
    # Go to rest position
    # motion_service.rest()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    head_ros = MoveHead(parser)
    main(session)
