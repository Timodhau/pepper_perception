import qi
import argparse
import sys
import time
import vision_definitions
from PIL import Image
import cv2 as cv
import numpy

def main(session):
    """
    This is just an example script that shows how images can be accessed
    through ALVideoDevice in Python.
    Nothing interesting is done with the images in this example.
    """
    # Get the service ALVideoDevice.

    video_service = session.service("ALVideoDevice")

    # Register a Generic Video Module
    resolution = vision_definitions.kQQVGA
    colorSpace = vision_definitions.kYUVColorSpace
    fps = 20
    nameId = video_service.subscribe("python_GVM", resolution, colorSpace, fps)
    video_service.unsubscribe(nameId)
    nameId = video_service.subscribe("python_GVM", resolution, colorSpace, fps)
    print 'getting images in remote'
    for i in range(0, 20):
        print "getting image " + str(i)
        naoImage = video_service.getImageRemote(nameId)
        # Get the image size and pixel array.
        imageWidth = naoImage[0]
        imageHeight = naoImage[1]
        array = naoImage[6]
        image_string = str(bytearray(array))

        # Create a PIL Image from our pixel array.
        pil_image = Image.fromstring("RGB", (imageWidth, imageHeight), image_string)
        open_cv_image = numpy.array(pil_image)
	# Convert RGB to BGR
	open_cv_image = open_cv_image[:, :, ::-1].copy()
        #pil_image.show()
	cv.imshow("Image window", open_cv_image)
        cv.waitKey(3)
    video_service.unsubscribe(nameId)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="10.0.1.8",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    main(session)