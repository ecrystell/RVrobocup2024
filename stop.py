from robot import Robot
import time
import cv2
robot = Robot('/dev/serial0')
# ~ r.move(0,0)

# 0 up 180 down
# ~ r.grabber(0, 180)
# ~ time.sleep(3)
# ~ r.grabber(180, 0)
# ~ r.grabber(0, 90)
# ~ r.grabber(90, 0)

# ~ cap = cv2.VideoCapture(0)
# ~ resolution = (320, 240)
# ~ cap.set(3, resolution[0])
# ~ cap.set(4, resolution[0])
# ~ cap.set(cv2.CAP_PROP_AUTO_WB,0.0)

# ~ while True:
	# ~ _, image = cap.read()
	# ~ image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	# ~ Blackline= cv2.inRange(image, (0), (50))
	
	# ~ cv2.imshow('blak', Blackline)
	# ~ cv2.waitKey(1)

robot.movedegrees(-90, -90, 40)
robot.movedegrees(-90, 90, 19)
robot.move(0,0)
time.sleep(5)
