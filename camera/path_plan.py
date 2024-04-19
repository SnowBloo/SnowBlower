import math
import random

import paho.mqtt.publish as publish
import time

HOSTNAME = "192.168.1.22"

class PathFinding:
    def __init__(self, position, angle):
        self.pointsDistance = 50
        self.destinationPoints = []
        self.sendData = []
        self.initVariables(position, angle)
        self.position = position

    def initVariables(self):
        self.initBox()
        self.initRobot()
        self.initStart()
        self.initPath()

    def initBox(self):
        # input destination points here
        self.boxPoints = [
            (0, 0),  # top left
            (800, 0),  # top right
            (800, 800),  # bottom right
            (0, 800)   # bottom left
        ]

    def initStart(self):
        # Get the initial position of the robot
        iterator = self.position

        # Get the direction vector towards the first box point
        direction = (self.boxPoints[0][0] - iterator[0], self.boxPoints[0][1] - iterator[1])

        # Calculate the distance
        distance = math.sqrt(direction[0] * direction[0] + direction[1] * direction[1])

        # If the distance is very small, consider the robot has reached the point
        if distance < 1.0:
            return

        # Calculate the angle
        angle = math.atan2(direction[1], direction[0]) * 180 / math.pi

        # Normalize the angle to be in the range [0, 360)
        angle = angle % 360

        # While distance is greater than 10 pixels, keep adding destination points
        while distance > self.pointsDistance:
            self.destinationPoints.append(iterator)

            # Update the iterator position
            iterator = (iterator[0] + self.pointsDistance * math.cos(angle * math.pi / 180),
                        iterator[1] + self.pointsDistance * math.sin(angle * math.pi / 180))

            # Recalculate the direction vector and distance
            direction = (self.boxPoints[0][0] - iterator[0], self.boxPoints[0][1] - iterator[1])
            distance = math.sqrt(direction[0] * direction[0] + direction[1] * direction[1])

    def initPath(self):
        # dimensions of destination box
        minX = self.boxPoints[0][0]  # Left edge
        minY = self.boxPoints[0][1]  # Top edge
        maxX = self.boxPoints[2][0]  # Right edge
        maxY = self.boxPoints[2][1]  # Bottom edge

        x, y = self.boxPoints[0][0], self.boxPoints[0][1]
        while y < maxY + self.pointsDistance:
            while x < maxX:
                self.destinationPoints.append((x, y))
                x += self.pointsDistance

            self.destinationPoints.append((x, y))
            y += self.pointsDistance

            while x > minX:
                self.destinationPoints.append((x, y))
                x -= self.pointsDistance

            self.destinationPoints.append((x, y))
            y += self.pointsDistance

    def initRobot(self):
        # input robot x & y positions here
        self.robot = self.position

    def setPosition(self, position):
        self.position = position
    
    def getPosition(self):
        return self.position
    
    def setAngle(self, angle):
        self.angle = angle

    def getAngle(self):
        return self.angle
    
    def main(self):
        for point in self.destinationPoints:
            self.moveToPoint(point)
            self.wait(10)

    def moveToPoint(self, point):
        # Calculate the vector from the robot to the target point
        direction = (point[0] - self.getPosition()[0], point[1] - self.getPosition()[1])

        # Calculate the distance
        distance = math.sqrt(direction[0] * direction[0] + direction[1] * direction[1])

        # Calculate the angle
        target_angle = math.atan2(direction[1], direction[0]) * 180 / math.pi + 180

        # Normalize the angle to be in the range [0, 360)
        target_angle = target_angle % 360

        # Calculate the angle difference between current rotation and target angle
        angleDiff = target_angle - self.getAngle()

        # Normalize the angle difference to be in the range [-180, 180)
        if angleDiff >= 180:
            angleDiff -= 360
        elif angleDiff < -180:
            angleDiff += 360

        # Apply a simple proportional control for turning
        # turnSpeed = 1.0  # Adjust this value to control turning speed
        # if abs(angleDiff) > turnSpeed:
            # Turn towards the target point
        if angleDiff > 0:
            while (angleDiff > 0):
                angleDiff = target_angle - self.getAngle()
                self.turnRight()
        else:
            while (angleDiff < 0):
                angleDiff = target_angle - self.getAngle()
                self.turnLeft()
        
        while (distance > 10):
            distance = math.sqrt(direction[0] * direction[0] + direction[1] * direction[1])
            self.moveForward()

    def moveForward(self):
        publish.single("movement", "forward", hostname=HOSTNAME)

    def moveBackward(self):
        publish.single("movement", "backward", hostname=HOSTNAME)

    def turnRight(self):
        publish.single("movement", "right", hostname=HOSTNAME)

    def turnLeft(self):
        publish.single("movement", "left", hostname=HOSTNAME)

    def wait(self):
        publish.single("movement", "wait", hostname=HOSTNAME)
    