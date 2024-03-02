# from picamera.array import PiRGBArray
# from picamera import PiCamera
# import RPi.GPIO as GPIO
import time
import cv2
import numpy as np


# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(40, GPIO.OUT)
# GPIO.output(40, GPIO.HIGH)

camera = cv2.VideoCapture(0) # webcam
resolution = (320, 240)
camera.set(3, resolution[0]) 
camera.set(4, resolution[0]) 
### IF USING RASPI UNCOMMENT LINES 16 TO 19
# camera = PiCamera()
# ~ camera.resolution = (640, 360)
# camera.rotation = 180
#rawCapture = PiRGBArray(camera, size=(640, 360))
time.sleep(0.1)

# for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True): ## UNCOMMENT THIS FOR RASPI
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

	centerx_blk = 0

	if len(contours_grn) > 0 :
			
		# drawing rect around the green square
		x_grn, y_grn , w_grn, h_grn = cv2.boundingRect(contours_grn[0])
		print(x_grn, y_grn, w_grn, h_grn)
		centerx_grn = int(x_grn + (w_grn/2))

		# drawing line in center of green square 
		cv2.line(image, (centerx_grn, 200), (centerx_grn, 250),(0,0,255),3)	

		# check if green is behind black line
		checkimage = image[100:200-y_grn,  x_grn:(x_grn+w_grn)]
		checkGreen = cv2.inRange(checkimage, (0,0,0), (50,50,50))
		checkGreen = cv2.erode(checkGreen, kernel, iterations=5)
		checkGreen = cv2.dilate(checkGreen, kernel, iterations=9)
		contours_chk, hierarchy_chk = cv2.findContours(checkGreen.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		
		if len(contours_chk)> 0:
			Greendected = True
			# im not gonna lie i dunno why the checkgreen works so if it doesnt send help
			# xchk, ychk , wchk, hchk = cv2.boundingRect(contours_chk[0])
			# centerx_chk = int(xchk + (wchk/2))  	   
			# cv2.line(image, (centerx_chk, 100), (centerx_chk, 200-y_grn),(0,255,0),3)

	if len(contours_blk) > 0 :
		x_blk, y_blk , w_blk, h_blk = cv2.boundingRect(contours_blk[0])
		centerx_blk = int(x_blk + (w_blk/2))  	   
		cv2.line(image, (centerx_blk, 200), (centerx_blk, 250),(255,0,0),3)

	if Greendected : 


		# if centerx_grn > (centerx_blk + 20):
		# 	cv2.putText(image, "Turn Right", (350,180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0),3)
		# elif centerx_grn < (centerx_blk - 20) :
		# 	cv2.putText(image, "Turn Left", (50,180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0),3)
		# else:
		# 	cv2.putText(image, "U-turn", (180,180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0),3)
		
		if centerx_grn > (centerx_blk + 20):
			cv2.putText(image, "Turn Right", (350,180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0),3)
		elif centerx_grn < (centerx_blk - 20) :
			cv2.putText(image, "Turn Left", (50,180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0),3)

	else :       
		# pd controller kind of
		setpoint = 320
		error = centerx_blk - setpoint
		centertext = "Error = " + str(error)
		cv2.putText(image, centertext, (200,340), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,0,0),3)	

	cv2.imshow("original with line", image)		
	#rawCapture.truncate(0)	### UNCOMMENT THIS FOR RASPI
	key = cv2.waitKey(1) & 0xFF	
	if key == ord("q"):
		break

# GPIO.output(40, GPIO.LOW) ### UNCOMMENT THIS FOR RASPI (off the cam)
