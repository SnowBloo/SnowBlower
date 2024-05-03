import math
import random

import paho.mqtt.publish as publish
import time


class MQTT:
    def __init__(self, hostname):
        self.HOSTNAME = hostname

    def send_data(self, left, right):
        publish.single("control", str(left) + "," + str(right), hostname=self.HOSTNAME)

    def stop(self):
        publish.single("control", "0,0", hostname=self.HOSTNAME)