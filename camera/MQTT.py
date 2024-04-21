import math
import random

import paho.mqtt.publish as publish
import time


class MQTT:
    def __init__(self, hostname):
        self.HOSTNAME = hostname

    def moveForward(self):
        publish.single("movement", "forward", hostname=self.HOSTNAME)

    def moveBackward(self):
        publish.single("movement", "backward", hostname=self.HOSTNAME)

    def turnRight(self):
        publish.single("movement", "right", hostname=self.HOSTNAME)

    def turnLeft(self):
        publish.single("movement", "left", hostname=self.HOSTNAME)

    def wait(self):
        publish.single("movement", "wait", hostname=self.HOSTNAME)
    