#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: PoseInit - Small example to make Nao go to an initial position."""

import qi
from naoqi import ALProxy
import sys
import time
import rospy
from std_msgs.msg import Bool


class PluginsPepper:
    def __init__(self, session):

        # parameters
        stiffnessLists = 1.0
        timeLists = 1.0
        names = "Body"
        name = "All"
        enable = False

        # Subscribers
        self.is_speaking_sub = rospy.Subscriber('/internal_state/is_speaking', Bool, self.callback_is_speaking)

        self.session = session
        # Get the services ALMotion & ALRobotPosture.
        self.motion_service = self.session.service("ALMotion")
        self.posture_service = self.session.service("ALRobotPosture")
        self.motion_service.setTangentialSecurityDistance(0)
        print(self.motion_service.getTangentialSecurityDistance())
        self.motion_service.setOrthogonalSecurityDistance(0)
        print(self.motion_service.getOrthogonalSecurityDistance())
        # Example showing how to activate "Move", "LArm" and "RArm" external anti collision
        self.motion_service.setExternalCollisionProtectionEnabled(name, enable)
        # Wake up robot
        self.motion_service.wakeUp()
        self.posture_service.goToPosture("StandInit", 0.5)
        time.sleep(0.5)
        self.motion_service.setAngles("HeadPitch", -0.45, 0.1)
        time.sleep(0.1)
        self.motion_service.setAngles("HeadYaw", 0., 0.1)
        time.sleep(0.1)
        # We use the "Body" name to signify the collection of all joints
        self.motion_service.stiffnessInterpolation(names, stiffnessLists, timeLists)
        # Get the service ALTabletService.
        self.tabletService = self.session.service("ALTabletService")
        self.tts = ALProxy('ALTextToSpeech', nao_ip, 9559)
        self.tts.setLanguage("English")

    def reset_position(self):
        self.motion_service.setAngles("HeadPitch", -0.45, 0.1)
        time.sleep(0.1)
        self.motion_service.setAngles("HeadYaw", 0., 0.1)
        time.sleep(3)

    def callback_is_speaking(self, data):
        if data.data:
            self.tabletService.showImage("http://198.18.0.1/img/help_charger.png")
        else:
            self.tabletService.hideImage()


if __name__ == "__main__":
    nao_ip = rospy.get_param("/nao_ip")
    nao_port = rospy.get_param("/nao_port")
    session = qi.Session()
    try:
        session.connect("tcp://" + nao_ip + ":" + str(nao_port))
    except RuntimeError:
        print("Can't connect to Naoqi at ip \"" + nao_ip + "\" on port " + str(nao_port) + ".\n"
                                                                                             "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    rospy.init_node('plugins_pepper', anonymous=False)
    plugin_pepper = PluginsPepper(session)
    while not rospy.is_shutdown():
        plugin_pepper.reset_position()
