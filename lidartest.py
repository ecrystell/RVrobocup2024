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

# ~ lidar.visualise(0, 180)
# ~ time.sleep(2)
while True:
	_, original = cap.read()
	image = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
	black = cv2.inRange(image, (0), (45))
	# ~ kernel = np.ones((3,3), np.uint8) 
	# ~ black = cv2.erode(black, kernel, iterations=5) 
	# ~ black = cv2.dilate(black, kernel, iterations=9)
	contours,_ = cv2.findContours(black.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
	sorted(contours, key=cv2.contourArea)
	x,y,w,h = cv2.boundingRect(contours[-1])
	cv2.rectangle(black, (x,y), (x+w, y+h), (255, 255, 0), 2)
	cv2.imshow('b', black)
	cv2.waitKey(1)
	# ~ print(lidar.getReading(140))
