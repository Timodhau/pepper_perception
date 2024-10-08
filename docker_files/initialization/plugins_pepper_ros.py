#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: PoseInit - Small example to make Nao go to an initial position."""

import qi
from naoqi import ALProxy
import argparse
import sys
import time
import rospy
from std_msgs.msg import Bool, Int64


class PluginsPepper:
    def __init__(self, session):

        # parameters
        stiffnessLists = 1.0
        timeLists = 1.0
        names = "Body"
        name = "All"
        enable = False
        self.HEAD_ANGLE_Y = -0.10
        self.HEAD_ANGLE_X = 0.00

        # Internal state
        self.in_conversation = False
        self.processing_llm = False

        # Subscribers
        self.is_speaking_sub = rospy.Subscriber('/internal_state/is_speaking', Bool, self.callback_is_speaking)
        # 0 wait 1 watch tower 2 engage 3 continue
        self.current_action = rospy.Subscriber('/action/engage_id', Int64, self.callback_current_action)


        self.in_conversation_sub = rospy.Subscriber('/internal_state/in_conversation', Bool,
                                                    self.callback_in_conversation)
        self.processing_llm_sub = rospy.Subscriber('/internal_state/processing_llm', Bool, self.callback_processing_llm)
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
        self.motion_service.setAngles("HeadPitch", self.HEAD_ANGLE_Y, 0.1)
        time.sleep(0.1)
        self.motion_service.setAngles("HeadYaw", self.HEAD_ANGLE_X, 0.1)
        time.sleep(0.1)
        # We use the "Body" name to signify the collection of all joints
        self.motion_service.stiffnessInterpolation(names, stiffnessLists, timeLists)
        # Get the service ALTabletService.
        self.tabletService = self.session.service("ALTabletService")
        self.tts = ALProxy('ALTextToSpeech', args.ip, 9559)
        # self.tts.setLanguage("English")
        self.tts.setLanguage("French")
        # self.html_content = f'''<html><head></head><body><p>Attendre que je finisse de parler</p></body></html>'''*
        self.tabletService.showImage("http://198.18.0.1/img/mute.png")

    def reset_position(self):
        # change if the robot is interacting
        if self.in_conversation:
            self.motion_service.setAngles("HeadPitch", self.HEAD_ANGLE_Y, 0.1)
        else:
            self.motion_service.setAngles("HeadPitch", self.HEAD_ANGLE_Y, 0.1)
        time.sleep(0.1)
        self.motion_service.setAngles("HeadYaw", self.HEAD_ANGLE_X, 0.1)
        time.sleep(3)

    def callback_is_speaking(self, data):
        # if self.processing_llm or data.data:
        #     self.tabletService.showImage("http://198.18.0.1/img/unmute.png")
            # Show HTML content on Pepper's tablet
            # self.tabletService.showWebview("data:text/html," + self.html_content)
        if not data.data:
            # mettre can answer
            self.tabletService.showImage("http://198.18.0.1/img/unmute.png")
            # self.tabletService.hideImage()

    def callback_processing_llm(self, data):
        self.processing_llm = data.data
        if self.processing_llm:
            self.tabletService.showImage("http://198.18.0.1/img/mute.png")

    def callback_current_action(self,data_action):
        pass
        """
        if data_action.data == -1 and self.watchtower_position == 0:
            # back in watchtower
            pass
        if data_action.data == -1 and self.watchtower_position == 1:
            # waiting in watchtower
            pass
        if data_action.data >= 0:
            # engaging
            pass
        """


    def callback_in_conversation(self, data):
        self.in_conversation = data.data

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="169.254.15.172",
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
    rospy.init_node('plugins_pepper', anonymous=False)
    plugin_pepper = PluginsPepper(session)
    while not rospy.is_shutdown():
        plugin_pepper.reset_position()
