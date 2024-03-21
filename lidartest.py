from LD19 import LD19
from robot import Robot
import time
import serial
import numpy as np
import pygame
import math
import cv2

lidar = LD19('/dev/ttyAMA3', offsetdeg = 0, flip = True) #offsetddeg was -90
robot = Robot('/dev/serial0')
robot.grabber(180, 0)
cap = cv2.VideoCapture(0)
resolution = (320, 240)
cap.set(3, resolution[0]) 
cap.set(4, resolution[0]) 

lidar.visualise(0, 180)
time.sleep(2)

def findBall(laserdata, lines):
	ballpts = []
	for i in range(45,135):
		x, y = laserdata[i]
		online = False
		for A,B,C in lines:
			dist = lidar.point_line_distance([x,y], A,B,C)
			
			if dist < 15:
				online = True
				break
		if online == False:
			ballpts.append(i)
			
	# ~ print(ballpts)
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
				if dist > 1:
					ball = (ballstart+ballend)//2
					return ball
				else:
					ballstart = ballend
	return -1

def pickUpBall():
	ball = -1
	samples = 12
	maxdist = 17
	noofpoints = 6
	direction = -1
	laserdata = lidar.getLaserXY()
	lines = lidar.RANSAC(laserdata, samples, maxdist, noofpoints)
	ball = findBall(laserdata, lines)
	lidar.ball[0] = ball
	print(ball)

	if lidar.getReading(10) > lidar.getReading(170):
		direction = -1
	else:
		direction = 1

	ballFound = False
	prevcenterx = -1
	while True:
		laserdata = lidar.getLaserXY()
		lines = lidar.RANSAC(laserdata, samples, maxdist, noofpoints)
		ball = findBall(laserdata, lines)
		lidar.ball[0] = ball
		_, original = cap.read()
		image = original
		
		if ball == -1:
			print('looking for ball')
			robot.move(direction*35, direction*35*-1)
			print(ball)

		elif not(ball < 100 and ball > 80) and prevcenterx == -1:
			print('turning to ball')
			speed = ball-90
			if abs(ball-90) < 30:
				if ball-90 > 0:
					speed = 30
				else:
					speed = -30
			
			robot.move(speed, -speed)

		else: # ball is on lidar detection and in front of robot
			print('seeing for ball')
			gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
			gray = cv2.medianBlur(gray, 5)

			
			circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT,1,100,param1=50,param2=30,minRadius=0,maxRadius=0)
			kp = 1.3

			if circles is not None:
				circles = np.uint16(np.around(circles))
				print("circle seen")
				for i in circles[0, :]:
					print(i[2])
					if i[2] > 90:
						#ballseen = True
						print("ball found")
						ballFound = True
						prevcenterx = i[0]
						center = (i[0], i[1])
						# circle center
						cv2.circle(image, center, 1, (0, 100, 100), 3)
						# circle outline
						radius = i[2]
						cv2.circle(image, center, radius, (255, 0, 255), 3)	
						break
			
			if prevcenterx == -1 or lidar.getReading(ball) > 50:
				robot.move(50, 50)
			else:
				error = 160 - prevcenterx
				
				robot.move(50+error*kp, 50-error*kp)
			
			if lidar.getReading(ball) < 10:
				robot.move(0,0)
				if ballFound:
					robot.grabber(0, 180)
					robot.grabber(180, 0)
					break
				else:
					robot.movedegrees(-70, -70, 50)
					robot.movedegrees(-70, 70, 50)
					prevcenterx = -1
			
		cv2.imshow("detected circles", image)
		if cv2.waitKey(1) == ord('q'):
			break

