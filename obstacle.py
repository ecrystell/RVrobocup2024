import time
import cv2
import numpy as np
import serial
from robot import *
from LD19 import LD19
import functools

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
kp = 2  # was 0.8
kd = 0


r = Robot('/dev/serial0')
lidar = LD19('/dev/ttyAMA3', offsetdeg=0, flip=True)  # offsetddeg was -90
Greendected = False
x_min = 160
y_min = 160
uturn = False
turnleft = False
turnright = False
error = 0


def checkObstacle():
	obsThreshold = 45 #lidar reading 
	minDeg = min(lidar.lidarvalues[80:100]) #take degree of lidar with smallest distance 
	if minDeg < obsThreshold:
		r.move(0, 0)
		print('obstacle detected')
		#r.movedegrees(-50,-50,20)
		leftAvg = sum(lidar.lidarvalues[35:55]) / 20
		rightAvg = sum(lidar.lidarvalues[125:145]) / 20
		if leftAvg > rightAvg:
			# uh hard code turn left
			print('moving left')
			r.movedegrees(-50, 50, 15.5)
			while True:
				print('curvingg')
				r.move(100, 40)
				_, image = cap.read()
				image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
				Blackline = cv2.inRange(image, (0), (55))
				cont, _ = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
				if len(cont) > 0:
					r.move(0, 0)
					break

		else:
			# hard code pls
			print('moving right')
			r.movedegrees(50, -50, 15.5)
			while True:
				print('curvingg')
				r.move(40, 100)
				_, image = cap.read()
				image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
				Blackline = cv2.inRange(image, (0), (55))
				cont, _ = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
				if len(cont) > 0:
					r.move(0, 0)
					break
					
r.move(50,50)
while True : 
	checkObstacle()
