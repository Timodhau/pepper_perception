import qi

session = qi.Session()
session.connect("tcp://10.0.1.3:9559")
motion_service = session.service("ALMotion")
posture_service = session.service("ALRobotPosture")
motion_service.wakeUp()
posture_service.goToPosture("StandInit", 0.5)
motion_service.setBreathEnabled('Body', True)


