#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Walk - Small example to make Nao walk"""

import qi
import argparse
import sys
import time
import random


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
    motion_service.setAngles("HeadPitch",  -0.45, 0.1)
    # stop move on the next double support
    # Go to rest position
    # motion_service.rest()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="169.254.112.44",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) + ".\n"
                                                                                             "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    main(session)
