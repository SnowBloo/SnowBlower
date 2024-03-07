# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import apriltag

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 60
rawCapture = PiRGBArray(camera, size=(640, 480))

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

def image_processing(image, results):
    centers = []
    for r in results:
        centers.append((int(r.center[0]), int(r.center[1])))
        
    # draw_center_box(image, centers)

    center = middle(centers)
    
    cv2.circle(image, center, 5, (0, 0, 255), -1)

    draw_tag(image, results)
	
    if len(centers) == 2:
        if determine_orientation(results):
            cv2.putText(image, "forward", (center[0], center[1] - 15),
                cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 2)
        else:
            cv2.putText(image, "backward", (center[0], center[1] - 15),
                cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 2)
	
# allow the camera to warmup
time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

	image = frame.array
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	results = detector.detect(gray)
	
	image_processing(image, results)

	cv2.imshow("Image", image)
	key = cv2.waitKey(1) & 0xFF
	
	rawCapture.truncate(0)

	if key == ord("q"):
		break
	
