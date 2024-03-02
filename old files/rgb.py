import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import serial
from robot import*

cap = cv2.VideoCapture(0)
resolution = (320, 240)
cap.set(3, resolution[0]) 
cap.set(4, resolution[0]) 

time.sleep(2)

x_last = resolution[0]/2
y_last = resolution[0]/2
invert = False
grayscale = False
run = False
time.sleep(1)
speed = 50
kp = 0.8
r = Robot('/dev/serial0')
Greendected = False

while True:
	
	_, original = cap.read()
	image = original
	roi = image[200:201, 0:320]
	f = open("rgbvalues.txt", 'a')
	f.write(str(roi[0]))
	f.close()
	cv2.imshow("orginal with line", roi)	
	key = cv2.waitKey(1) & 0xFF	
	if key == ord("q"): #q: exit program 
		r.move(0, 0)
		break

