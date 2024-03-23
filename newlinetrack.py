import time
import cv2
import numpy as np
import serial
from robot import *
from LD19 import LD19

cap = cv2.VideoCapture(0)
resolution = (320, 240)
cap.set(3, resolution[0])
cap.set(4, resolution[0])

time.sleep(2)

x_last = resolution[0] / 2
y_last = resolution[0] / 2
invert = False
grayscale = False
run = False
time.sleep(1)
speed = 90
kp = 2.5  # was 0.8
kd = 0
k = 4


r = Robot('/dev/serial0')
lidar = LD19('/dev/ttyAMA3', offsetdeg=0, flip=True)  # offsetddeg was -90
Greendected = False
x_min = 160
y_min = 160
uturn = False
turnleft = False
turnright = False
error = 0


def clamp(n, smallest, largest):
	return max(smallest, min(n, largest))


def checkObstacle():
	obsThreshold = 55 #lidar reading 
	minDeg = min(lidar.lidarvalues[80:100]) #take degree of lidar with smallest distance 
	if minDeg < obsThreshold:
		r.move(0, 0)
		print('obstacle detected')
		r.movedegrees(-50,-50,5)
		leftAvg = sum(lidar.lidarvalues[35:55]) / 20
		rightAvg = sum(lidar.lidarvalues[125:145]) / 20
		if leftAvg > rightAvg:
			# uh hard code turn left
			print('moving left')
			r.movedegrees(-50, 50, 15)
			# ~ r.movedegrees(50, 50, 10)
			r.move(100, 70)
			time.sleep(3)
			while True:
				print('curvingg')
				r.move(100, 70)
				
				_, image = cap.read()
				image = cv2.cvtColor(image[220:320, 0:320], cv2.COLOR_BGR2GRAY)
				Blackline = cv2.inRange(image, (0), (55))
				kernel = np.ones((3, 3), np.uint8)
				Blackline = cv2.erode(Blackline, kernel, iterations=5)
				Blackline = cv2.dilate(Blackline, kernel, iterations=9)
				cv2.imshow("obs", Blackline)
				cont, _ = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
				if len(cont) > 0:
					r.move(0, 0)
					
					break

		else:
			# hard code pls
			print('moving right')
			r.movedegrees(50, -50, 15)
			r.move(70, 100)
			time.sleep(3)
			while True:
				print('curvingg')
				r.move(70, 100)
				
				_, image = cap.read()
				image = cv2.cvtColor(image[220:320, 0:320], cv2.COLOR_BGR2GRAY)
				Blackline = cv2.inRange(image, (0), (55))
				kernel = np.ones((3, 3), np.uint8)
				Blackline = cv2.erode(Blackline, kernel, iterations=5)
				Blackline = cv2.dilate(Blackline, kernel, iterations=9)
				cont, _ = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
				cv2.imshow("obs", Blackline)
				if len(cont) > 0:
					r.move(0, 0)
					
					break

prevError = 0
#r.grabber(0, 180)
while True:
	checkObstacle()
	_, original = cap.read()

	image = original
	Greendected = False
	if grayscale:
		image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		Blackline = cv2.inRange(image, (0), (55))
	else:
		image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		bottom = image[220:320, 0:320]
		top = image[120:220, 0:320]
		grnimg = image[120:320, 0:320]
		bottomBlack = cv2.inRange(bottom, (0, 0, 0), (255, 255, 70))
		topBlack = cv2.inRange(top, (0, 0, 0), (255, 255, 70))
		topGreen = cv2.inRange(grnimg, (40, 110, 80), (100, 255, 255))

		kernel = np.ones((3, 3), np.uint8)  # to get the RGB thingies
		topGreen = cv2.erode(topGreen, kernel, iterations=5)  # eroding and dilating
		topGreen = cv2.dilate(topGreen, kernel, iterations=9)

		Redline = cv2.inRange(top, (0, 150, 100), (20, 255, 255))
		Redline = cv2.erode(Redline, kernel, iterations=5)  # eroding and dilating
		Redline = cv2.dilate(Redline, kernel, iterations=9)
		contours_red, _ = cv2.findContours(Redline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		if len(contours_red) > 0:  # see red line stop n break
			r.move(0, 0)
			print('red')
			run = False

	kernel = np.ones((3, 3), np.uint8)
	topBlack = cv2.erode(topBlack, kernel, iterations=5)
	topBlack = cv2.dilate(topBlack, kernel, iterations=9)
	bottomBlack = cv2.erode(bottomBlack, kernel, iterations=5)
	bottomBlack = cv2.dilate(bottomBlack, kernel, iterations=9)

	if invert:
		Blackline = cv2.bitwise_not(Blackline)
	contours_blkTop, _ = cv2.findContours(topBlack.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	contours_blkBtm, _ = cv2.findContours(bottomBlack.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	contours_grn, _ = cv2.findContours(topGreen.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	if len(contours_blkTop) > 0 or len(contours_blkBtm) > 0:
		errors = []
		if len(contours_blkTop) == 0:
			# ~ print('top nothing')
			contoursToCheck = [contours_blkBtm, ]
			x, y, w, h = cv2.boundingRect(contours_blkBtm[0])
			centerx = int(x + (w // 2))
			cv2.line(image, (centerx, 250), (centerx, 300), (255, 0, 0), 3)
			setpoint = resolution[0] / 2
			error = int(centerx - setpoint)
			#print(error)
			errors.append(error)
		elif len(contours_blkBtm) == 0:
			# ~ print('bottom nothing')
			contoursToCheck = [contours_blkTop, ]
			x, y, w, h = cv2.boundingRect(contours_blkTop[0])
			centerx = int(x + (w // 2))
			cv2.line(image, (centerx, 150), (centerx, 200), (255, 0, 0), 3)
			setpoint = resolution[0] / 2
			error = int(centerx - setpoint)
			#print(error)
			errors.append(error)
		else:
			# ~ print('both something!')
			contoursToCheck = [contours_blkTop, contours_blkBtm]
			x, y, w, h = cv2.boundingRect(contours_blkTop[0])
			centerx = int(x + (w // 2))
			cv2.line(image, (centerx, 150), (centerx, 200), (255, 0, 0), 3)
			setpoint = resolution[0]  /2
			error = int(centerx - setpoint)          
			#print(error)
			errors.append(error)

			x, y, w, h = cv2.boundingRect(contours_blkBtm[0])
			centerx = int(x + (w // 2))
			cv2.line(image, (centerx, 250), (centerx, 300), (255, 0, 0), 3)
			setpoint = resolution[0] / 2
			error = int(centerx - setpoint)
			#print(error)
			errors.append(error)

		if len(errors) > 0:
			if len(errors) == 1:
				error = errors[0]
			else:
				if abs(errors[0]) > abs(errors[1]):
					error = errors[1]
				else:
					error = errors[0]
			
		# ~ cv2.putText(image, str(error), (10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
		if run:
			#print("running")
			P = error * kp
			D = (error - prevError) * kd
			
			if error != 0:
				left = int(abs(speed/error)*k  + (P+D))
				right = int(abs(speed/error)*k - (P+D))
				print("error", error, "left", left, "right", right)
				r.move(clamp(int(abs(speed/error)*k  + (P+D)), -255, 255), clamp(int(abs(speed/error)*k - (P+D)), -255, 255))
			else:
				r.move(clamp(int(speed + (P+D)), -255, 255), clamp(int((speed) - (P+D)), -255, 255))
			
		else:
			r.move(0, 0)
		prevError = error
	else:
		#print('where line')
		pass
	if len(contours_grn) > 0:
		# compare sizes of contours, get all contours of a certain size and check y of contours
		# if more than 1 rectangle then uturn
		# drawing rect around the green square
		rects = []
		for contour in contours_grn:
			x_grn, y_grn, w_grn, h_grn = cv2.boundingRect(contour)
			area = h_grn * w_grn
			if area > 2000 and y_grn > 50:
				rects.append([h_grn*w_grn, x_grn, y_grn, w_grn, h_grn])
				
		sorted(rects, reverse = True)
		print(rects)
		
		
		if len(rects) > 0:
			area, x_grn,y_grn,w_grn,h_grn = rects[0]

			cv2.rectangle(image, (x_grn, y_grn + 120), (x_grn + w_grn, y_grn + h_grn + 120), (255, 0, 0), 2)
			print(' see square')
			centerx_grn = int(x_grn + (w_grn / 2))
			# drawing line in center of green square
			cv2.line(image, (centerx_grn, 200), (centerx_grn, 250), (0, 0, 255), 3)
			
			error = 0
			
			# check if green is behind black line
			if centerx_grn < centerx:
				checkimage = grnimg[y_grn - 30:y_grn, x_grn:(x_grn + w_grn//2)]
			else:
				checkimage = grnimg[y_grn - 30:y_grn, (x_grn + w_grn//2):(x_grn + w_grn)]
			checkGreen = cv2.inRange(checkimage, (0, 0, 0), (255, 255, 70))
			checkGreen = cv2.erode(checkGreen, kernel, iterations=5)
			checkGreen = cv2.dilate(checkGreen, kernel, iterations=9)
			contours_chk, hierarchy_chk = cv2.findContours(checkGreen.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

			if len(contours_chk) > 0:
				if len(rects) == 2:
					
					x_grn1 = rects[1][1]
					#if x_grn - 20 < x_grn1 < x_grn + 20:
					cv2.putText(image, "U-turn", (50, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
					uturn = True
					Greendected = False
					# ~ else:
						# ~ Greendected = True
				else:
					Greendected = True

				# ~ cv2.rectangle(image, (x_grn, y_grn), (x_grn + w_grn, y_grn + h_grn), (255, 0, 0), 2)

	cv2.imshow("orginal with line", image)
	key = cv2.waitKey(1) & 0xFF
	if Greendected:
		Greendected = False
		if centerx_grn > (centerx):
			cv2.putText(image, "Turn Right", (50, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
			turnright = True
		elif centerx_grn < (centerx):
			cv2.putText(image, "Turn Left", (50, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
			turnleft = True
		if run:
			if uturn:
				r.movedegrees(60, 60, 15)
				r.movedegrees(-60, 60, 32)
				r.movedegrees(-60, -60, 5)
				print('uturn')
				time.sleep(5)
				uturn = False

			elif turnright:
				r.movedegrees(60, 60, 17)
				r.movedegrees(60, -60, 16)
				r.movedegrees(-60, -60, 2)
				print('turnright')
				time.sleep(5)
				turnright = False
			elif turnleft:
				r.movedegrees(60, 60, 17)
				r.movedegrees(-60, 60, 16)
				r.movedegrees(-60, -60, 2)
				print('turn left')
				time.sleep(5)
				turnleft = False

	#cv2.imshow("top", topBlack)
	#cv2.imshow("btotom", bottomBlack)


	# keys to interact with code
	if key == ord("q"):  # q: exit program
		r.move(0, 0)
		break
	elif key == ord("r"):  # r: toggle run mode
		run = not run
	elif key == ord("e"):  # e: increase speed by 5
		speed = speed + 5
		print("speed:" + str(speed) + "\tkp:" + str(kp))
	elif key == ord("w"):  # w: decrease speed by 5
		speed = speed - 5
		print("speed:" + str(speed) + "\tkp:" + str(kp))
	elif key == ord("a"):  # a: increase kp ny 0.5
		kp = kp + 0.05
		print("speed:" + str(speed) + "\tkp:" + str(kp))
	elif key == ord("s"):  # s: decrease kp by 0.5
		kp = kp - 0.05
		print("speed:" + str(speed) + "\tkp:" + str(kp))
	elif key == ord("j"):  # j: increase kp ny 0.5
		k = k + 0.1
		print("speed:" + str(speed) + "\tk:" + str(k))
	elif key == ord("k"):  # k: decrease kp by 0.5
		k = k - 0.1
		print("speed:" + str(speed) + "\tk:" + str(k))

