from naoqi import ALProxy
import requests
import json
import os
import time
import pwd

with open("/Pepper/cfg.json") as json_file:
	cfg = json.load(json_file)

tts = ALProxy('ALTextToSpeech',10.0.1.8,9559)
tts.setLanguage("French")
