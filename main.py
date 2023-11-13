# from picamera.array import PiRGBArray
# from picamera import PiCamera
# import RPi.GPIO as GPIO
import time
import cv2
import numpy as np


# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(40, GPIO.OUT)
# GPIO.output(40, GPIO.HIGH)

# camera = PiCamera()
camera = cv2.VideoCapture(0)
# camera.resolution = (640, 360)
# camera.rotation = 180
#rawCapture = PiRGBArray(camera, size=(640, 360))
time.sleep(0.1)

# for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):	
while True:
	Greendected = False # check whether got green

	# getting the frame from vid capture
	image = camera.read()[1]

	# cutting image to only one part
	roi = image[200:250, 0:639]

	# finding green and black line
	Blackline = cv2.inRange(roi, (0,0,0), (50,50,50))
	Greensign = cv2.inRange(roi, (0,65,0), (100,200,100))

	kernel = np.ones((3,3), np.uint8) # kernel for erosion and dilation

	# erode and dilate to remove noise (like threshold so it dun false detect)
	Blackline = cv2.erode(Blackline, kernel, iterations=5)
	Blackline = cv2.dilate(Blackline, kernel, iterations=9)
	Greensign = cv2.erode(Greensign, kernel, iterations=5)
	Greensign = cv2.dilate(Greensign, kernel, iterations=9)	

	# finding contours of black and green
	contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	contours_grn, hierarchy_grn = cv2.findContours(Greensign.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	if len(contours_grn) > 0 :
		Greendected = True	
		# drawing rect around the green square
		x_grn, y_grn , w_grn, h_grn = cv2.boundingRect(contours_grn[0])
		centerx_grn = int(x_grn + (w_grn/2))

		# drawing line in center of green square 
		cv2.line(image, (centerx_grn, 200), (centerx_grn, 250),(0,0,255),3)	
	if len(contours_blk) > 0 :
		x_blk, y_blk , w_blk, h_blk = cv2.boundingRect(contours_blk[0])
		centerx_blk = int(x_blk + (w_blk/2))  	   
		cv2.line(image, (centerx_blk, 200), (centerx_blk, 250),(255,0,0),3)
	if Greendected : 
		if centerx_grn > centerx_blk :
			cv2.putText(image, "Turn Right", (350,180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0),3)
		else :
			cv2.putText(image, "Turn Left", (50,180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0),3)
	else :       
		# pd controller kind of
		setpoint = 320
		error = centerx_blk - setpoint
		centertext = "Error = " + str(error)
		cv2.putText(image, centertext, (200,340), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,0,0),3)	

	cv2.imshow("original with line", image)		
	#rawCapture.truncate(0)	
	key = cv2.waitKey(1) & 0xFF	
	if key == ord("q"):
		break

# GPIO.output(40, GPIO.LOW)