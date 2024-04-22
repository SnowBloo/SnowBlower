import math
import random

import paho.mqtt.publish as publish
import time


class MQTT:
    def __init__(self, hostname):
        self.HOSTNAME = hostname

    def set_left(self, value):
        publish.single("left", value, hostname=self.HOSTNAME)

    def set_right(self, value):
        publish.single("right", value, hostname=self.HOSTNAME)
