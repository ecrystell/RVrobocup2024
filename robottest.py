import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
from robot import Robot

robot = Robot('/dev/serial0')

robot.ramp(133) 
# ~ robot.sorter(30)
# ~ robot.grabber(180, 0)
# ~ time.sleep(0.1)
# ~ robot.grabber(0,180)

# ~ for i in range(0, 360, 5):
	# ~ robot.grabber(i-5, i)
	# ~ time.sleep(0.5)
	# ~ print(i)
	
