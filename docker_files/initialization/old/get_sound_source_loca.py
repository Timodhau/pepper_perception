#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: A Simple class to get & read FaceDetected Events"""

import qi
import time
import sys
import argparse


class SoundSourceLoca(object):
    """
    A simple class to react to face detection events.
    """

    def __init__(self, app):
        """
        Initialisation of qi framework and event detection.
        """
        super(SoundSourceLoca, self).__init__()
        app.start()
        session = app.session
        # raise sensitivity
        self.sound_detect_service = session.service("ALSoundDetection")
        # Sets the sensitivity of the detection to 0.3 (less sensitive than default).
        # The default value is 0.9.
        self.sound_detect_service.setParameter("Sensitivity", 1.0)
        # Get the service ALMemory.
        self.memory = session.service("ALMemory")
        # Connect the event callback.
        self.subscriber = self.memory.subscriber("ALSoundLocalization/SoundLocated")
        self.subscriber.signal.connect(self.on_sound_source)


    def on_sound_source(self, value):
        """
        Callback for event FaceDetected.
        """
        print(value)

    def run(self):
        """
        Loop on, wait for events until manual interruption.
        """
        print "Starting HumanGreeter"
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print "Interrupted by user, stopping HumanGreeter"
            self.subscriber.unsubscribe("ALSoundLocalization/SoundLocated")
            #stop
            sys.exit(0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="10.0.1.3",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    try:
        # Initialize qi framework.
        connection_url = "tcp://" + args.ip + ":" + str(args.port)
        app = qi.Application(["HumanGreeter", "--qi-url=" + connection_url])
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    sound_source_loca = SoundSourceLoca(app)
    sound_source_loca.run()