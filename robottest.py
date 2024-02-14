import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
from robot import Robot

robot = Robot('/dev/serial0')
print("robot initiatlised")

while True:
	robot.light(150, 0, 0)
	print("lights on")
	time.sleep(1)
	robot.light(0, 150, 0)
	time.sleep(1)
	robot.light(0, 0, 150)
	time.sleep(1)
	robot.light(150, 150, 150)
	time.sleep(1)
	#robot.move(-100, 100)
	time.sleep(1)
	robot.move(0,0)
	time.sleep(1)
	robot.movedegrees(100, 100, 50)
	time.sleep(1)
	#robot.movedegrees(-100, -100, 50)
	time.sleep(1)
