import paho.mqtt.publish as publish
import time

HOSTNAME = "192.168.1.22"

for i in range(10):
    publish.single("test", str(i), hostname=HOSTNAME)
    time.sleep(1)

