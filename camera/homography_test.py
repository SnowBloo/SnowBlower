
import time
import cv2
import math

from skimage import transform
import numpy as np
import matplotlib.pyplot as plt

def homography_test(image):
	src = np.array([391, 100, 
					14, 271,
					347, 624,
					747, 298,]).reshape((4, 2))#destination coordinates
	dst = np.array([100, 100, 
					100, 650,
					650, 650,
					650, 100,]).reshape((4, 2))

	tform = transform.estimate_transform('projective', src, dst)
	tf_image = transform.warp(image, tform.inverse, output_shape=(768, 1376))	

	cv2.imwrite("homography_test.jpg", tf_image)

	return tf_image
	
# allow the camera to warmup
time.sleep(0.1)

while True:
	ret, frame = cv2.VideoCapture(0).read()

	image = frame.array
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	cv2.imshow("Image", image)
	key = cv2.waitKey(1) & 0xFF

	if key == ord("q"):
		break
	
	if key == ord("t"):
		tf_image = homography_test(image)

		fig, ax = plt.subplots()
		ax.imshow(tf_image)
		_ = ax.set_title('projective transformation')
		
	