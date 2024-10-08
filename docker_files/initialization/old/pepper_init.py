#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Use wakeUp Method"""

import qi
import argparse
import sys
import time
import pwd
import os
import json

def main(session):
    """
    This example uses the wakeUp method.
    """
    # Get the service ALMotion.
   
    motion_service  = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture") 

    motion_service.wakeUp()
    posture_service.goToPosture("StandInit", 0.5)
    
    motion_service.setBreathEnabled('Body', True)
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="pepper.local",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    # load config file
    with open("./cfg.json") as json_file:
        cfg = json.load(json_file)
    session = qi.Session()
    try:
        session.connect("tcp://" + cfg['network']['robot_ip'] + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    main(session)
