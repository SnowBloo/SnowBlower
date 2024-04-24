# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import apriltag
import math
import argparse

import io
import numpy as np
import itertools
import sys

from scipy.spatial import distance

from MQTT import MQTT

if len(sys.argv) < 2:
	print("Usage: python3 detection.py <ip address>")
	exit()

hostname = sys.argv[1]

flipped = False

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (1280, 960)
camera.framerate = 60
rawCapture = PiRGBArray(camera, size=(1280, 960))

options = apriltag.DetectorOptions(families='tag36h11',
								 border=1,
								 nthreads=4,
								 quad_decimate=1.0,
								 quad_blur=0.0,
								 refine_edges=True,
								 refine_decode=False,
								 refine_pose=False,
								 debug=False,
								 quad_contours=True)

detector = apriltag.Detector(options)

ANGLE_OFFSET = 0

def draw_tag(image, results):
	for r in results:
		(ptA, ptB, ptC, ptD) = r.corners
		ptB = (int(ptB[0]), int(ptB[1]))
		ptC = (int(ptC[0]), int(ptC[1]))
		ptD = (int(ptD[0]), int(ptD[1]))
		ptA = (int(ptA[0]), int(ptA[1]))
		
		cv2.line(image, ptA, ptB, (0, 255, 0), 2)
		cv2.line(image, ptB, ptC, (0, 255, 0), 2)
		cv2.line(image, ptC, ptD, (0, 255, 0), 2)
		cv2.line(image, ptD, ptA, (0, 255, 0), 2)
		
		tagID = r.tag_id
		(cX, cY) = (int(r.center[0]), int(r.center[1]))
		cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
		cv2.putText(image, str(tagID), (cX, cY - 15),
			cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
		
		tagFamily = r.tag_family.decode("utf-8")
		cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
			cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
	
def draw_center_box(image, centers):
	if len(centers) != 4:
		return 
	cv2.line(image, centers[0], centers[1], (0, 0, 255), 2)
	cv2.line(image, centers[1], centers[3], (0, 0, 255), 2)
	cv2.line(image, centers[3], centers[2], (0, 0, 255), 2)
	cv2.line(image, centers[2], centers[0], (0, 0, 255), 2)

def image_processing(image, results):
	if len(results) == 0:
		return []
	centers = []
	for r in results:
		centers.append((int(r.center[0]), int(r.center[1])))
	
	center = np.mean(centers, axis = 0, dtype=int)
	
	cv2.circle(image, (center[0], center[1]), 5, (0, 0, 255), -1)

	draw_tag(image, results)
	
	if len(centers) == 2:
		cv2.line(image, centers[0], centers[1], (0, 0, 255), 2)
		
		cv2.text(image, "Angle: " + str(calculate_angle(centers)), (center[0], center[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
		# cv2.text(image, "Distance: " + str(math.dist(centers[0], centers[1])), (center[0], center[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
			
	return centers

def sortpts_clockwise(A):
    # Sort A based on Y(col-2) coordinates
    sortedAc2 = A[np.argsort(A[:,1]),:]

    # Get top two and bottom two points
    top2 = sortedAc2[0:2,:]
    bottom2 = sortedAc2[2:,:]

    # Sort top2 points to have the first row as the top-left one
    sortedtop2c1 = top2[np.argsort(top2[:,0]),:]
    top_left = sortedtop2c1[0,:]

    # Use top left point as pivot & calculate sq-euclidean dist against
    # bottom2 points & thus get bottom-right, bottom-left sequentially
    sqdists = distance.cdist(top_left[None], bottom2, 'sqeuclidean')
    rest2 = bottom2[np.argsort(np.max(sqdists,0))[::-1],:]

    # Concatenate all these points for the final output
    return np.concatenate((sortedtop2c1,rest2),axis =0)

def convert_to_homo(pt, h):
	source_coord = np.array([pt[0], pt[1], 1])
	homo_pt = np.matmul(h, source_coord)

	homo_pt = homo_pt / homo_pt[2]

	return (int(homo_pt[0]), int(homo_pt[1]))

def calculate_angle(centers):
	x_dist = centers[0][0] - centers[1][0]
	y_dist = centers[0][1] - centers[1][1]

	angle = math.atan2(y_dist, x_dist) * 180 / math.pi

	return angle - ANGLE_OFFSET

def init_path(points_offset):
	points = []

	minX = 0
	minY = 0
	maxX = 800
	maxY = 800
	
	for y, x in itertools.product(range(minY, maxY + 1, points_offset), range(minX, maxX + 1, points_offset)):
		points.append((x, y))

	return points

homo = True
# allow the camera to warmup
time.sleep(0.1)

camera.capture("coord_image.jpg")
coord_image = cv2.imread("coord_image.jpg")
gray = cv2.cvtColor(coord_image, cv2.COLOR_BGR2GRAY)

if flipped:
	coord_image = cv2.flip(coord_image, 0)
	gray = cv2.flip(gray, 0)

results = detector.detect(gray)

draw_tag(coord_image, results)

src = np.empty(0)
numClicks = 0
def draw_circle(event,x,y,flags,param):
	if event == cv2.EVENT_LBUTTONDBLCLK:
		global src, numClicks
		numClicks = numClicks + 1
		cv2.circle(coord_image,(x,y),5,(255,0,0),-1)
		src = np.append(src, [x, y])

cv2.namedWindow('image')
cv2.setMouseCallback('image',draw_circle)

while True:
	cv2.imshow('image',coord_image)
	k = cv2.waitKey(20) & 0xFF
	if k == 27 or k == ord('q') or numClicks == 4:
		cv2.destroyAllWindows()
		break
	elif k == ord('a'):
		print(src)

src = src.reshape((4, 2))
src = sortpts_clockwise(src)

dst = np.array([0, 0,
				800, 0,
				800, 800,
				0, 800,]).reshape((4, 2))

robot = MQTT(hostname)

points = init_path(50) 
i = 0

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array
	
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	if flipped:
		gray = cv2.flip(gray, 0)
		image = cv2.flip(image, 0)
	results = detector.detect(gray)
	
	centers = image_processing(image, results)

	if homo:
		h, status = cv2.findHomography(src, dst)
		homo_img = cv2.warpPerspective(image, h, (800, 800))
		
		centers_homo = list(np.array([convert_to_homo(center, h) for center in centers]))
		
		if len(centers_homo) < 2:
			continue

		robot_center = np.mean(centers_homo, axis = 0, dtype = int)
		angle = calculate_angle(centers_homo)

		point = points[i]
		direction = (point[0] - robot_center[0], point[1] - robot_center[1])

		robot_distance = math.dist(point, robot_center)

		target_angle = math.atan2(direction[1], direction[0]) * 180 / math.pi

		angleDiff = target_angle - angle

		desired_speed = 60 
		angular_speed_multiplier = 4
		forward_speed_multiplier = 0.8

		angular_speed = angular_speed_multiplier * angleDiff
		forward_speed = desired_speed - forward_speed_multiplier * angleDiff
		
		left_speed = forward_speed - angular_speed
		right_speed = forward_speed + angular_speed # TODO: adjust and tune parameters
		
		robot.set_left(left_speed)
		robot.set_right(right_speed)

		if robot_distance < 10:
			if i < len(points):
				i += 1 			# move to next point
			else: 
				# it's done
				camera.close()
				cv2.destroyAllWindows()
				break

		for center in centers_homo:
			cv2.circle(homo_img, (center[0], center[1]), 4, (255, 0, 0), -1)

		cv2.circle(homo_img, (robot_center[0], robot_center[1]), 4, (255, 0, 0), -1)

		cv2.imshow("homo", homo_img)
	
	cv2.imshow("Image", image)
	key = cv2.waitKey(1) & 0xFF
	rawCapture.truncate(0)
	if key == ord("q"):
		camera.close()
		cv2.destroyAllWindows()
		break

	if key == ord("w"):
		ANGLE_OFFSET = 0
		ANGLE_OFFSET = calculate_angle(centers_homo)
	
	if key == ord("t"):
		homo = not homo

	if key == ord("d"): # debug info
		print("====debug info====")
		print("Angle Offset: " + ANGLE_OFFSET)
		print("Current Angle: " + angle)
		print("Target Angle: " + target_angle)
		print("Distance: " + robot_distance)
		print("==================")

