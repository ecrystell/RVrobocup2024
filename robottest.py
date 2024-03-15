import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
from robot import Robot

robot = Robot('/dev/serial0')
print("robot initiatlised")
cap = cv2.VideoCapture(0)
resolution = (320, 240)
cap.set(3, resolution[0]) 
cap.set(4, resolution[0]) 

while True:
	_, original = cap.read()
	image = original
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray = cv2.medianBlur(gray, 5)
	rows = gray.shape[0]
	circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT,1,100,param1=50,param2=30,minRadius=0,maxRadius=0)

	if circles is not None:
		circles = np.uint16(np.around(circles))
		
		print("ball seen")
		for i in circles[0, :]:
			center = (i[0], i[1])
			# circle center
			cv2.circle(gray, center, 1, (0, 100, 100), 3)
			# circle outline
			radius = i[2]
			cv2.circle(image, center, radius, (255, 0, 255), 3)	
			print(i[0])
			if radius > 100:
				ballseen = True
	cv2.imshow("detected circles", image)
	if cv2.waitKey(1) == ord('q'):
		break
# ~ def moveGrabber(curr, target):
	# ~ if target > curr:
		# ~ for i in range(curr, target,3):
			# ~ robot.grabber(i)
			# ~ time.sleep(0.05)
	# ~ else:
		# ~ for i in range(curr, target, -3):
			# ~ robot.grabber(i)
			# ~ time.sleep(0.05)
# ~ curr = 0
# ~ robot.grabber(0)
# ~ while True:
	# ~ robot.move(0,0)
	# ~ robot.light(150, 0, 0)
	# ~ print("lights on")
	# ~ time.sleep(1)
	# ~ robot.light(0, 150, 0)
	# ~ time.sleep(1)
	
	
	# ~ moveGrabber(0, 180)

	# ~ time.sleep(2)
	# ~ moveGrabber(180, 0)

#robot.movedegrees(-100, -100, 50)
#robot.grabber(20)
#time.sleep(2)
