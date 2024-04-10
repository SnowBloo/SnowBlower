import paho.mqtt.publish as publish
import time

HOSTNAME = "10.168.84.51"

for i in range(10):
    publish.single("number", str(i), hostname=HOSTNAME)
    time.sleep(1)

