import time
import cv2
import numpy as np
import serial
from robot import *
from LD19 import LD19

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_AUTO_WB,0.0)
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

while True:
	_, image = cap.read()
	silver = cv2.inRange(image, (27, 15, 62), (255, 112, 122))

	kernel = np.ones((3, 3), np.uint8)  # to get the RGB thingies
	silver = cv2.erode(silver, kernel, iterations=5)  # eroding and dilating
	silver = cv2.dilate(silver, kernel, iterations=9)
	contours_silv, _ = cv2.findContours(silver.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	if len(contours_silv) > 0:
		silverbox = cv2.minAreaRect(contours_silv[0])
		(x,y), (w,h), ang = silverbox
		if w*h >= 5000:
			box = cv2.boxPoints(silverbox)
			box = np.int0(box)
			cv2.drawContours(image, [box], 0, (255,0,0), 3)
			# ~ cv2.imshow('hh', silverbox)
	cv2.imshow('silver', image)
	cv2.waitKey(1) & 0xFF
