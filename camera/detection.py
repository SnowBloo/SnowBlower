# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import apriltag
import math

from skimage import transform
from scipy.spatial import distance
import numpy as np

FLIPPED = True	

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

def middle(centers):
	if len(centers) == 0:
		return (0, 0)
	# calculate the centerpoint of the centers
	x = 0
	y = 0
	for c in centers:
		x += c[0]
		y += c[1]
		
	x = int(x / len(centers))
	y = int(y / len(centers))
	
	return (x, y)

def determine_orientation(results):
	return results[0].tag_id == 0

def line_length(centers):	
	if len(centers) != 2:
		return 0
	
	return math.sqrt((centers[0][0] - centers[1][0])**2 + (centers[0][1] - centers[1][1])**2)

def determine_distance(centers):
		length = line_length(centers)
		if length == 0:
			return 0
		return 728.5 / length

def image_processing(image, results):
	if len(results) == 0:
		return []
	centers = []
	for r in results:
		centers.append((int(r.center[0]), int(r.center[1])))
	
	center = middle(centers)
	
	cv2.circle(image, center, 5, (0, 0, 255), -1)

	draw_tag(image, results)
	
	cv2.putText(image, "distance: " + str(determine_distance(centers)), (int(center[0]), int(center[1] - 30)),
		cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 2)
	if len(centers) == 2:
		cv2.line(image, centers[0], centers[1], (0, 0, 255), 2)
		cv2.putText(image, "length: " + str(line_length(centers)), (int(centers[0][0]), int(centers[0][1] - 15)),
			cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 2)
		
		if determine_orientation(results):
			cv2.putText(image, "forward", (center[0], center[1] - 15),
				cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 2)
		else:
			cv2.putText(image, "backward", (center[0], center[1] - 15),
				cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 2)
			
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

homo = False
# allow the camera to warmup
time.sleep(0.1)

camera.capture("coord_image.jpg")
coord_image = cv2.imread("coord_image.jpg")

if FLIPPED:
	coord_image = cv2.flip(coord_image, 0)

global mouseX, mouseY, src

src =  np.empty(0)
# numClicks = 0
def draw_circle(event,x,y,flags,param):
	global src
	if event == cv2.EVENT_LBUTTONDBLCLK:
		# numClicks = numClicks + 1
		cv2.circle(coord_image,(x,y),5,(255,0,0),-1)
		src = np.append(src, [x, y])

cv2.namedWindow('image')
cv2.setMouseCallback('image',draw_circle)

while(True):
	cv2.imshow('image',coord_image)
	k = cv2.waitKey(20) & 0xFF
	if k == 27 or k == ord('q'):
		break
	elif k == ord('a'):
		print(src)

src = src.reshape((4, 2))

print(src)
src = sortpts_clockwise(src)
src = src + 10
print(src)
dst = np.array([100, 100,
				500, 100,
				500, 500,
				100, 500,]).reshape((4, 2))

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

	image = frame.array
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	if FLIPPED:
		gray = cv2.flip(gray, 0)
		image = cv2.flip(image, 0)
	results = detector.detect(gray)
	
	centers = image_processing(image, results)

	if homo:
		h, status = cv2.findHomography(src, dst)
		homo_img = cv2.warpPerspective(image, h, (400, 400))
		cv2.imshow("homo", homo_img)
	
	cv2.imshow("Image", image)
	key = cv2.waitKey(1) & 0xFF

	rawCapture.truncate(0)

	if key == ord("q"):
		camera.close()
		break
	
	if key == ord("t"):
		homo = not homo

	if key == ord("c"):
		distance = str(input("input distance: "))
		
		cv2.imwrite("distance_" + distance + " ft_" + "length_" + str(line_length(centers)) + " px.jpg" , image)

