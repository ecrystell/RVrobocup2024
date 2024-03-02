import time
import cv2
import numpy as np

camera = cv2.VideoCapture(0) # webcam
resolution = (320, 240)
camera.set(3, resolution[0]) 
camera.set(4, resolution[1]) 

time.sleep(0.1)

while True:
	# getting the frame from vid capture
	image = camera.read()[1]

	print(image[120][160]) # DETECTING RGB VALUE IN THE MIDDLE
	cv2.imshow("collecting rgb values", image)	
	key = cv2.waitKey(1) & 0xFF	
	if key == ord("q"):
		break
		#test
