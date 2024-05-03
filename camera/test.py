from MQTT import MQTT
import time

robot=MQTT("10.168.93.73")

robot.send_data(0, 0)