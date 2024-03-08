from LD19 import LD19
from robot import Robot
import cv2
import time
import serial
import numpy as np
import math

def findeqns(x1, y1, x2, y2):
	gradient = (y2-y1) / (x2-x1)
	yintercept = y1 - (gradient * x1)
	return (gradient, yintercept)
	
lidar = LD19('/dev/ttyAMA3', offsetdeg = 0, flip = True) #offsetddeg was -90
robot = Robot('/dev/serial0')
lidar.visualise(0, 180)
time.sleep(2)
img = cv2.imread("screenshot.jpg")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 50, 200)
# cv2.imshow("edges", edges)
lines = cv2.HoughLines(edges, 3, np.pi/180, 110)
weirdpts = []
start = 0

if lines is None:
	print("no img")
else:
	for line in lines:
		# ~ x1,x2,y1,y2 = line[0]
		
		rho,theta = line[0]
		a = np.cos(theta)
		b = np.sin(theta)
		x0 = a*rho
		y0 = b*rho
		x1 = int(x0 + 1000*(-b))
		y1 = int(y0 + 1000*(a))
		x2 = int(x0 - 1000*(-b))
		y2 = int(y0 - 1000*(a))
		
		cv2.line(img,(x1,y1), (x2,y2), (0,255,0), 2)
		gradient, yintercept = findeqns(x1,y1,x2,y2)
		for i in range(start, 180):
			dist = lidar.lidarvalues[i]
			pointx = (dist * 0.3 * math.cos(i/360 * 2 * math.pi + (math.pi))) + 360
			pointy = (dist * 0.3 * math.sin(i/360 * 2 * math.pi + (math.pi))) + 360
			
			perpenddist = abs(( gradient * pointx - pointy + yintercept)/(gradient**2 +1) ** 0.5)
			if perpenddist > 10:
				if i - start < 20:
					weirdpts.append(i)
				start = i
				break

cv2.imshow("img", img)
cv2.waitKey(0)
#cv2.imshow("lines", lines)
	

	
	

# ~ while True:
	# ~ for i in range(179):
		# ~ print(i, lidar.getReading(i))
	# pointgroup = []
	# pointcount = 0
	# for i in range(179):
		# distance = lidar.getReading(i)-lidar.getReading(i+1)
		# if distance < 30:
			# pointcount = pointcount + 1
		# else:
			# if pointcount < 8 and len(pointgroup) > 0:
				# pointgroup[-1] += pointcount
			# else:
				# pointgroup.append(pointcount)
			# pointcount = 0
	#print(pointgroup)
	#print(lidar.getReading(90))
	# try:	
		# lidarwall = lidar.getReading(140)
		# lidarstop = lidar.getReading(90)
		# error = lidarwall - 300
		# P = error * 0.3
		# if lidarstop < 150:
			# robot.move(0,0)
		# else :
			# robot.move(60+P, 60-P)
	# except KeyboardInterrupt:
		# robot.move(0,0)
		# break
    # values = [0] * 12
    # for i in range (12):
        # values[i] = lidar.getReading(180+ i*12)
    # print(values)
