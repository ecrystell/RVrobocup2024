from LD19 import LD19
from robot import *
import time
import serial
import numpy as np
import pygame
import math
import cv2

# ~ lidar = LD19('/dev/ttyAMA3', offsetdeg = 0, flip = True) #offsetddeg was -90
# ~ robot = Robot('/dev/serial0')
# ~ #robot.grabber(180, 180)
# ~ cap = cv2.VideoCapture(0)
# ~ resolution = (320, 240)
# ~ cap.set(3, resolution[0]) 
# ~ cap.set(4, resolution[0]) 

# ~ 
# ~ time.sleep(2)
# ~ green = False
# ~ red = False

def findCenter(threshold):
	robot.movedegrees(100, 100, 50)
	robot.move(70, -70)
	runtime = time.time() + 3
	while time.time() < runtime:
		if lidar.getReading(90) < threshold:
			diff = lidar.getReading(90) - threshold
			robot.move(70 - (0.1 * diff), -70)
			
def findBall(lidar, laserdata, lines):
	ballpts = []
	for i in range(45,135):
		x, y = laserdata[i]
		online = False
		for A,B,C in lines:
			dist = lidar.point_line_distance([x,y], A,B,C)
			
			if dist < 30:
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

def pickUpBall(cap, lidar, robot):
	start = time.time()
	lidar.visualise(0, 180)
	ball = -1
	samples = 12
	maxdist = 17
	noofpoints = 6
	direction = -1
	ballcount = 0
	laserdata = lidar.getLaserXY()
	lines = lidar.RANSAC(laserdata, samples, maxdist, noofpoints)
	ball = findBall(lidar, laserdata, lines)
	lidar.ball[0] = ball
	print(ball)
	

	if lidar.getReading(10) > lidar.getReading(170):
		direction = -1
	else:
		direction = 1

	ballFound = False
	prevcenterx = -1
	while time.time() < start+120:
		laserdata = lidar.getLaserXY()
		lines = lidar.RANSAC(laserdata, samples, maxdist, noofpoints)
		ball = findBall(lidar, laserdata, lines)
		lidar.ball[0] = ball
		_, original = cap.read()
		image = original
		
		if ball == -1:
			print('looking for ball')
			robot.move(direction*50, direction*50*-1)
			print(ball)

		elif not(ball < 100 and ball > 80) and prevcenterx == -1:
			print('turning to ball')
			speed = (ball-90)*1.5
			if abs(speed) < 50:
				if ball-90 > 0:
					speed = 50
				else:
					speed = -50
			
			robot.move(speed, -speed)

		else: # ball is on lidar detection and in front of robot
			print('seeing for ball')
			gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
			graycircle = cv2.medianBlur(gray, 5)

			
			circles = cv2.HoughCircles(graycircle, cv2.HOUGH_GRADIENT,1,100,param1=50,param2=30,minRadius=0,maxRadius=0)
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
						black = cv2.inRange(gray, (0), (50))
						contours, _ = cv2.findContours(black.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
						if len(contours) > 0:
							print('black ball')
							# turn sorter to black)
							robot.sorter(70)
						else:
							print('silver ball')
							# turn sorter to silver
							robot.sorter(30)
						prevcenterx = i[0]
						center = (i[0], i[1])
						# circle center
						cv2.circle(image, center, 1, (0, 100, 100), 3)
						# circle outline
						radius = i[2]
						cv2.circle(image, center, radius, (255, 0, 255), 3)	
						break
			
			if prevcenterx == -1 or lidar.getReading(ball) > 50:
				robot.move(90, 90)
			else:
				error = 160 - prevcenterx
				
				robot.move(90+error*kp, 90-error*kp)
			
			if lidar.getReading(ball) < 4:
				robot.move(0,0)
				if ballFound:
					robot.movedegrees(-90, -90, 25)
					robot.grabber(0, 180)
					robot.movedegrees(90, 90, 25)
					robot.grabber(180, 0)
					time.sleep(1)
					robot.grabber(0, 100)
					time.sleep(1)
					robot.grabber(100, 0)
					ballcount += 1
					
					if ballcount < 3:
						robot.movedegrees(-90, -90, 10)
						# ~ robot.movedegrees(-90, 90, 5)
						ball = -1
						prevcenterx = -1
					else:
						break
				else:
					robot.movedegrees(-90, -90, 30)
					robot.movedegrees(-90, 90, 50)
					prevcenterx = -1
			
		cv2.imshow("detected circles", image)
		if cv2.waitKey(1) == ord('q'):
			break
	robot.move(0,0)
	
	
def findTriangle(green, red):
	while True:
		_, original = cap.read()
		image = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)

		topGreen = cv2.inRange(image, (20, 130, 70), (90, 255, 255))

		kernel = np.ones((3, 3), np.uint8)  # to get the RGB thingies
		topGreen = cv2.erode(topGreen, kernel, iterations=5)  # eroding and dilating
		topGreen = cv2.dilate(topGreen, kernel, iterations=9)
		contours_grn, _ = cv2.findContours(topGreen.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		Redline = cv2.inRange(image, (160, 75, 115), (180, 220, 255))
		Redline = cv2.erode(Redline, kernel, iterations=5)  # eroding and dilating
		Redline = cv2.dilate(Redline, kernel, iterations=9)
		contours_red, _ = cv2.findContours(Redline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		if len(contours_grn) > 0 and not green:  # means haven't seen green triangle before
			# drawing rect around the green square
			x_grn, y_grn, w_grn, h_grn = cv2.boundingRect(contours_grn[0])
			centerx_grn = int(x_grn + (w_grn / 2))
			centery_grn = int(y_grn + (h_grn / 2))
			# drawing line in center of green square
			cv2.line(image, (centerx_grn, centery_grn), (centerx_grn, centery_grn), (0, 0, 255), 3)
			cv2.rectangle(image, (x_grn, y_grn), (x_grn + w_grn, y_grn + h_grn), (255, 0, 0), 2)
			setpoint = resolution[0] / 2
			error = int(centerx_grn - setpoint)
			if lidar.getReading(90) < 40:
				# drop the ball code
				robot.movedegrees(-70, 70, 45)
				robot.move(-50, -50)
				time.sleep(2)
				# uh drop ball
				green = True
				break
			else:
				robot.move(clamp(int(speed + error * kp), -255, 255), clamp(int(speed - error * kp), -255, 255))

		elif len(contours_red) > 0 and not red:
			# drawing rect around the green square
			x_red, y_red, w_red, h_red = cv2.boundingRect(contours_red[0])
			centerx_red = int(x_red + (w_red / 2))
			centery_red = int(y_red + (h_red / 2))
			# drawing line in center of green square
			cv2.line(image, (centerx_red, centery_red), (centerx_red, centery_red), (0, 0, 255), 3)
			cv2.rectangle(image, (x_red, y_red), (x_red + w_red, y_red + h_red), (255, 0, 0), 2)
			setpoint = resolution[0] / 2
			error = int(centerx_red - setpoint)
			if lidar.getReading(90) < 40:
				# drop the ball code
				robot.movedegrees(-70, 70, 45)
				robot.move(-50, -50)
				time.sleep(2)
				# uh drop ball
				red = True
				break
			else:
				robot.move(clamp(int(speed + error * kp), -255, 255), clamp(int(speed - error * kp), -255, 255))


		
def wallTrack(lidar, robot, threshold=80):
	
	print('start finding wall')
	closest = -1
	while closest < 88 or closest > 92:
		speed = (closest-90)*1.5
		if abs(speed) < 50:
			if closest-90 > 0:
				speed = 50
			else:
				speed = -50
		robot.move(speed, -speed)
		minreading = min(lidar.lidarvalues)
		for i in range(5,175):
			if lidar.lidarvalues[i] == minreading:
				print(minreading, " minreading at ", i)
				closest = i
				break
	
	while lidar.getReading(closest) > 100:
		print('moving to wall')
		robot.move(100,100)
	robot.move(0,0)
	robot.movedegrees(-90, 90, 19)
	print('start walltracking')
	return
	speed = 90
	kp = 1.5
	while True:
		_, original = cap.read()
		image = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)
		right = lidar.getReading(177)
		front = lidar.getReading(90)
		print(right, front)
		if right > 1150 or front > 1150:
			robot.move(0, 0)
			print('exit??')
			return
		if front < 120:
			robot.move(0,0)
			

			greenimg = cv2.inRange(image, (35, 150, 45), (70, 160, 160))

			kernel = np.ones((3, 3), np.uint8)  # to get the RGB thingies
			greenimg = cv2.erode(greenimg, kernel, iterations=5)  # eroding and dilating
			greenimg = cv2.dilate(greenimg, kernel, iterations=9)
			contours_grn, _ = cv2.findContours(greenimg.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

			redimg = cv2.inRange(image, (0, 120, 33), (50, 255, 255))
			redimg = cv2.erode(redimg, kernel, iterations=5)  # eroding and dilating
			redimg = cv2.dilate(redimg, kernel, iterations=9)
			contours_red, _ = cv2.findContours(redimg.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			if len(contours_grn) > 0 and not(green):
				print('green triangle')
				robot.move(0,0)
				return
			elif len(contours_red) > 0 and not(red):
				print('red triangle')
				robot.move(0,0)
				return
			else:
				print('corner')
				# ~ robot.movedegrees(-90, -90, 40)
				robot.movedegrees(-90, 90, 19)
		else:
			error = right - threshold
			robot.move(clamp(int(speed + error * kp), -255, 255), clamp(int(speed - error * kp), -255, 255))
		
		cv2.imshow('image', image)
		key = cv2.waitKey(1)
		

	


'''
evaczone
	1. enter zone
	2. look for ball
	3. pick up ball
	4. go center
	5. repeat 2 to 4
	6. (if camera is good enuf) look for triangle colours
	7. go to triangle n deposit ball
	8. go center
	9. repeat 6 and 7
	10. walltrack to find exit
'''
def test():
	silver = True
	distRobotToWall = 100
	lidar = LD19('/dev/ttyAMA3', offsetdeg = 0, flip = True) #offsetddeg was -90
	robot = Robot('/dev/serial0')
	#robot.grabber(180, 180)
	cap = cv2.VideoCapture(0)
	resolution = (320, 240)
	cap.set(3, resolution[0]) 
	cap.set(4, resolution[0]) 

	
	time.sleep(2)
	green = False
	red = False
	if silver:
		pickUpBall(cap, lidar, robot) # includes look for ball and pick up ball
		# ~ findCenter(distRobotToWall)
		# ~ pickUpBall()
		# ~ pickUpBall()
		# ~ findCenter(distRobotToWall)
		# ~ findTriangle()
		# ~ findCenter(distRobotToWall)
		# ~ findTriangle()
		# ~ findCenter()
		# ~ wallTrack()
		
# ~ test()
		
