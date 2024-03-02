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

while True:
		
	_, original = cap.read()
	
	image = original[0:270, 0:320]
	edges = cv2.Canny(image,100,200)
	cv2.imshow("edges", edges)
	cv2.imshow("image", image)
	
	key = cv2.waitKey(1) & 0xFF	
	
	#keys to interact with code 
	if key == ord("q"): #q: exit program 
		r.move(0, 0)
		break
