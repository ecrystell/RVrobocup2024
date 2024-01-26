from LD19 import LD19
from robot import Robot
import time
import serial


lidar = LD19('/dev/ttyAMA3', offsetdeg = -90, flip = True)
robot = Robot('/dev/serial0')
lidar.visualise(0, 180)
while False:
	print(lidar.getReading(90))
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
