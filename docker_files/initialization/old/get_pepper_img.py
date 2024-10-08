#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
"""example: Shows how images can be accessed through ALVideoDevice"""

import qi
import argparse
import sys
import rospy
from sensor_msgs.msg import Image as img
from cv_bridge import CvBridge
import vision_definitions
from PIL import Image
import cv2 as cv
import numpy


class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)

        # Publishers
        self.pub = rospy.Publisher("/pepper_robot/camera/head/image_raw", img, queue_size=10)

        # Subscribers
        # rospy.Subscriber("/camera/image_color",Image,self.callback)

    # def callback(self, msg):
    #    rospy.loginfo('Image received...')
    #    self.image = self.br.imgmsg_to_cv2(msg)

    def start(self):
        rospy.loginfo("Timing images")
        # rospy.spin()
        video_service = session.service("ALVideoDevice")

        # Register a Generic Video Module
        resolution = 2
        colorSpace = 11  # vision_definitions.kRGBColorSpace
        fps = 10
        nameId = video_service.subscribe("python_GVM", resolution, colorSpace, fps)
        video_service.unsubscribe(nameId)
        nameId = video_service.subscribe("python_GVM", resolution, colorSpace, fps)
        print'getting images in remote'
        while not rospy.is_shutdown():
            naoImage = video_service.getImageRemote(nameId)
            # Get the image size and pixel array.
            imageWidth = naoImage[0]
            imageHeight = naoImage[1]
            array = naoImage[6]
            image_string = str(bytearray(array))

            # Create a PIL Image from our pixel array.
            pil_image = Image.fromstring("RGB", (imageWidth, imageHeight), image_string)
            open_cv_image = numpy.array(pil_image)
            open_cv_image = open_cv_image[:, :, ::-1].copy()
            # open_cv_image = cv.cvtColor(open_cv_image, cv.COLOR_BGR2RGB)
            # print(open_cv_image.shape)
            self.pub.publish(self.br.cv2_to_imgmsg(open_cv_image, "bgr8"))
            # Convert RGB to BGR
            # pil_image.show()
            # cv.imshow("Image window", open_cv_image)
            # cv.waitKey(3)
        video_service.unsubscribe(nameId)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="10.0.1.3",
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
    rospy.init_node("imagetimer111", anonymous=True)
    my_node = Nodo()
    my_node.start()
