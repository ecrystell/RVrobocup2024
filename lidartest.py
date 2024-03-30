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
print(lidar.getReading(90))
