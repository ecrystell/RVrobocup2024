from LD19 import LD19
from robot import Robot
import time
import serial
import np
import pygame
import math
import cv2

lidar = LD19('/dev/ttyAMA3', offsetdeg = 0, flip = True) #offsetddeg was -90
robot = Robot('/dev/serial0')
cap = cv2.VideoCapture(0)
resolution = (320, 240)
cap.set(3, resolution[0]) 
cap.set(4, resolution[0]) 

time.sleep(2)

def findBall(laserdata, lines):
	ballpts = []
	for i in range(45,135):
		x, y = laserdata[i]
		online = False
		for A,B,C in lines:
			dist = lidar.point_line_distance([x,y], A,B,C)
			
			if dist < 5:
				online = True
				break
		if online == False:
			ballpts.append(i)
			
	print(ballpts)
	# use p turning to adjust ball to angle the ball is at
	if len(ballpts) > 1:
		ballstart = ballpts[0]
		ballend = 0
		ball = -1
		for i in range(len(ballpts)-1):
			if (ballpts[i] + 1 != ballpts[i+1]) or (i == len(ballpts)-2):
				if i == len(ballpts)-2:
					ballend = ballpts[-1]
				else:
					ballend = ballpts[i]
				x1, y1 = laserdata[ballstart]
				x2, y2 = laserdata[ballend]
				dist = lidar.distance([x1,y1],[x2,y2])
				# ~ print(dist)
				if dist > 1 and dist < 13.5:
					ball = (ballstart+ballend)//2
					return ball
				else:
					ballstart = ballend



laserdata = lidar.getLaserXY()
lines = lidar.RANSAC(laserdata, 15, 15, 14)
ball = findBall(laserdata, lines)
lidar.ball = ball
print(ball)

lidar.visualise(0, 180)
while not(ball < 95 and ball > 85):
    lidar.ball = ball
    robot.move(ball-90, -(ball-90))

ballseen = False
while not(ballseen):
	robot.move(30, 30)
	_, original = cap.read()

	image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray = cv2.medianBlur(gray, 5)

	rows = gray.shape[0]
	circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8)


	if circles is not None:
		circles = np.uint16(np.around(circles))
		ballseen = True
	for i in circles[0, :]:
		center = (i[0], i[1])
		# circle center
		cv2.circle(image, center, 1, (0, 100, 100), 3)
		# circle outline
		radius = i[2]
		cv2.circle(image, center, radius, (255, 0, 255), 3)
		

	cv2.imshow("detected circles", image)
	cv2.waitKey(0)
robot.move(0,0)