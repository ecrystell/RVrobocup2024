from robot import Robot
import time
import cv2
robot = Robot('/dev/serial0')
# ~ r.move(0,0)

robot.move(0,0)
time.sleep(5)
