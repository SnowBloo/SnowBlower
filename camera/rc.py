from MQTT import MQTT
import time

from pynput import keyboard
from pynput.keyboard import Key, Listener

robot=MQTT("10.168.93.73")

# this program is just to create an MCVE and to test the detection code. It's not an actual program.


held = 0  
right_speed, left_speed = 120, 120

once = True
def on_press(key):
    global right_speed, left_speed, once
    
    right_speed *= 1
    left_speed *= 1
    if abs(right_speed) > 120:
        right_speed = right_speed / abs(right_speed) * 120
    if abs(left_speed) > 120:
        left_speed = left_speed / abs(left_speed) * 120
    if key == Key.up:
        robot.send_data(int(right_speed), int(right_speed))
        # robot.send_data(120, 120)
    elif key == Key.down:
        if once:
            left_speed *= -1
            right_speed *= -1
            once = False
        robot.send_data(int(left_speed), int(left_speed))
        # robot.send_data(-120, -120)
    elif key == Key.left:
        if once:
            left_speed *= -1
            once = False
        robot.send_data(int(left_speed), int(right_speed))
        # robot.send_data(-120, 120)
    elif key == Key.right:
        if once:
            right_speed *= -1
            once = False
        robot.send_data(int(right_speed), int(left_speed))
        # robot.send_data(120, -120)
    print('Key: ', key, ' was held')


def on_release(key):
    global once, right_speed, left_speed
    once = True
    right_speed, left_speed = 120, 120
    robot.send_data(0, 0)
    print('Key: ', key, ' as released')

with Listener(on_press=on_press, on_release=on_release) as listener:
    print('listener starts')
    listener.join()