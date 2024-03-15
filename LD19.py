import time
import serial
import threading
import pygame
import math
import multiprocessing
import numpy as np

def convert (bytestream):
    lsb = bytestream[0]
    msb = bytestream[1]
    reading = (msb << 8) | lsb
    return reading

def process_stream(byte_stream):
    readings = []
    # Iterate through the stream in steps of 3 bytes
    for i in range(0, 36, 3):
        # Extract LSB, MSB, and intensity
        lsb = byte_stream[i]
        msb = byte_stream[i + 1]
        intensity = byte_stream[i + 2]

        # Combine LSB and MSB to get the reading
        reading = (msb << 8) | lsb

        # Append the reading and intensity to the list
        readings.append({'reading': reading, 'intensity': intensity})
    return readings

import math

def angle_between_points(x1, y1, x2, y2, x3, y3):

  # Calculate the vectors from the second point to the other two points
	v1 = (x1 - x2, y1 - y2)
	v2 = (x3 - x2, y3 - y2)

  # Calculate the dot product and magnitude of the vectors
	dot_product = v1[0] * v2[0] + v1[1] * v2[1]
	magnitude_v1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
	magnitude_v2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)

  # Prevent division by zero
	if magnitude_v1 == 0 or magnitude_v2 == 0:
		return 0

  # Apply the cosine rule to find the angle in radians
	angle_radians = math.acos(dot_product / (magnitude_v1 * magnitude_v2))

  # Convert the angle to degrees and return
	return math.degrees(angle_radians) % 360
        

class LD19:
	def __init__ (self, port, baud = 230400, offsetdeg = 0, flip = False):
		self.readings = [0]* 360
		self.lidarvalues = multiprocessing.Array('i',range(360))
		self.ball = multiprocessing.Array('i', range(1))
		self.queue = multiprocessing.Queue()
		self.port = port
		self.flip = flip
		self.offsetdeg = offsetdeg
		self.visualisation = False
		self.serial = serial.Serial(
			port=self.port,
			baudrate= baud,
			parity= serial.PARITY_NONE,
			stopbits= serial.STOPBITS_ONE,
			bytesize= serial.EIGHTBITS
		)
		if not self.serial:
			print("unable to initialize")
		self.readLidar = multiprocessing.Process(target = self.readData)
		self.readLidar.start()
		self.ss = 0
	def terminate(self):
		self.readLidar.terminate()
		self.visualisethread.terminate()
	
	def readData(self):
		while True:
			# Read a byte from the serial port
			byte_data = self.serial.read(1)
			# Check if any byte is received
			if byte_data:
				# Convert the byte to an integer
				value = int.from_bytes(byte_data, byteorder='big')
				# Compare with 0x54 (84 in decimal)
				if value == 0x54:
					byte_data = self.serial.read(1)
					VerLen = int.from_bytes(byte_data, byteorder='big')
					byte_data = self.serial.read(2)
					Speed = convert(byte_data)
					byte_data = self.serial.read(2)
					Startangle = convert(byte_data)
					readings = process_stream(self.serial.read(36))
					byte_data = self.serial.read(2)
					Endangle = convert(byte_data)
					byte_data = self.serial.read(2)
	
					Timestamp = convert(byte_data)
					byte_data = self.serial.read(1)
					CRC = int.from_bytes(byte_data, byteorder='big')

					step = (((Endangle + 36000) - (Startangle+36000)) % 36000)/(12 - 1)
					# ~ values = [0]*360
					for i in range (12):
						angle = (int((Startangle + i * step)/100) + self.offsetdeg + 360) % 360
						if self.flip:
							angle = int(359 - angle)
						self.lidarvalues[angle] = min(readings[i]['reading'], 1200) #
					

	def getReading(self, angle): # returns distance from the dot, one dot every degree
		return self.lidarvalues[angle]
	def visualise(self, startangle = 0, endangle = 180):
		
		if not self.visualisation:
			
			self.startangle = (startangle) % 360
			self.endangle = (endangle) % 360
			# starts a separate thread by itself when run
			self.visualisethread =  multiprocessing.Process(target = self.visualiseThread, args=(self.queue,))
			self.visualisethread.start()
		else:
			print("visualisation already started")
	def stopvisualise(self):
		if self.visualisation:
			self.visualisethread.terminate()
			self.visualisation = False
    
	def find_best_fit_line(self, points):
		# Convert the list of points into NumPy arrays
		points = np.array(points)
		
		# Compute the centroid of the points
		centroid = np.mean(points, axis=0)
		
		# Compute the covariance matrix of the points
		covariance_matrix = np.cov(points, rowvar=False)
		
		# Compute the eigenvalues and eigenvectors of the covariance matrix
		eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)
		
		# Sort the eigenvalues and corresponding eigenvectors in descending order
		sorted_indices = np.argsort(eigenvalues)[::-1]
		sorted_eigenvalues = eigenvalues[sorted_indices]
		sorted_eigenvectors = eigenvectors[:, sorted_indices]
		
		# Choose the eigenvector corresponding to the smallest eigenvalue as the direction of the line
		direction = sorted_eigenvectors[:, 1]  # Assuming the smallest eigenvalue corresponds to the second eigenvector
		
		# Compute the coefficients of the line equation (Ax + By + C = 0) using the direction vector and the centroid
		A, B = direction
		C = -(A * centroid[0] + B * centroid[1])
		return A, B, C

	def point_line_distance(self, point, A, B, C):
		# Calculate the distance from the point to the line using the formula
		distance = abs(A*point[0] + B*point[1] + C) / np.sqrt(A**2 + B**2)
		return distance

	def distance(self, point1, point2):
		return ((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)**0.5

	def RANSAC(self, laserdata, samples, maxDist, numberOfPoints=11): #samples is number of consec points to find best fit line
		
		lines = []
		laserdataLength = len(laserdata)
		associatedReadings = [0] * len(laserdata)
		startpoint = 0
		# ~ print(len(laserdata))
		startpts = []
		colours = ["cyan", "magenta", "yellow", "green", "pink", "purple", "red"]
		j=0
		while startpoint< len(laserdata):
			
			count = 0
			for i in range(startpoint, laserdataLength-1):
				# ~ print(i)
				if associatedReadings[i] == 0: 
					dist = self.distance(laserdata[i], laserdata[i+1])
					# ~ print(dist)
					if dist > 10:
						count = 0
					else:
						count += 1
					if count == samples:
						startpoint = i-samples +1
						# ~ print('hello')
						break
				if i == laserdataLength-1:
					startpoint = laserdataLength
					# ~ print('hello')
			startpts.append(startpoint)
			
			if startpoint+ samples > laserdataLength-1:
				# print("no more samples")
				break
			a, b, c = self.find_best_fit_line(laserdata[startpoint:startpoint+samples]) # a b c is vector coords
			numpointsonline = 0
			linepoints = []
			for i in range(startpoint, startpoint+samples):
				#print(self.point_line_distance(laserdata[i], a, b, c))
				if self.point_line_distance(laserdata[i], a, b, c) < maxDist: # check distance to line
					numpointsonline += 1
					linepoints.append(laserdata[i])
					associatedReadings[i] = 1
			#time.sleep(30)
			# ~ print(numpointsonline)
			if numpointsonline > numberOfPoints: # how many points u need to be on a line to be a line
				cluster = numberOfPoints
				clusterpoints = []
				tempassosciatedReadings = []
				for i in range(startpoint + samples, len(laserdata)):
					
					if self.point_line_distance(laserdata[i], a, b, c) < maxDist:
						if self.distance(laserdata[i], linepoints[-1]) > 20: #if first point of new cluster, far away from old cluster
						
							cluster = 0
							clusterpoints = []
							clusterpoints.append(laserdata[i])
							tempassosciatedReadings.append(i)
						else:   # if there is a new cluster, we want multiple points in close proximity 
							cluster += 1
							if cluster < 7: # if there is less than 3 in the cluster, it may not be considered as in the same line as the current best fit, so take notes of it temporaryily first
								tempassosciatedReadings.append(i)
								clusterpoints.append(laserdata[i])
							elif cluster == 7: # if cluster == 3, it is quite confirmed that it is on the line, add points and find new best fit
								clusterpoints.append(laserdata[i])
								tempassosciatedReadings.append(i)
								linepoints.extend(clusterpoints)
								for i in tempassosciatedReadings:
									associatedReadings[i] = 1
								a, b, c = self.find_best_fit_line(linepoints)
							else: #cluster more than 5 keep adding to line 
								linepoints.append(laserdata[i])
								associatedReadings[i] = 1
								a, b, c = self.find_best_fit_line(linepoints)
				
				if self.distance(linepoints[0], linepoints[-1])> 30: # check distance of the line, if its long enough
					a, b, c = self.find_best_fit_line(linepoints[:-1])
					# pygame.draw.circle(self.screen, colours[j], linepoints[0], 5)
					# pygame.draw.circle(self.screen, colours[j], linepoints[-2], 10)
					
					# j+=1
					# j %= len(colours)
					lines.append((a, b, c))
			startpoint += samples
		self.queue.put(lines)
		return lines
			
		
	def draw_lines(self, lines, color=(0, 0, 0), linewidth=2):
		center = (0,0)
		for A, B, C in lines:
			if B == 0:
                # Handle the case when B is zero (line is horizontal)
				x = -C / A
				pygame.draw.line(self.screen, color, (x, 0), (x, 720), linewidth)
			else:
				m = -A / B  # Slope
				b = -C / B  # y-intercept
				# Calculate two points on the line for drawing
				x1 = 0
				y1 = int(m * x1 + b)
				x2 = 720 - 1
				y2 = int(m * x2 + b)

				# Draw the line on the screen
				pygame.draw.line(self.screen, color, (x1 + center[0], y1 + center[1]), (x2 + center[0], y2 + center[1]), linewidth)
	
	def getLaserXY(self):
		laserdata = []
		for i in range(self.startangle, self.endangle):
			x = (self.lidarvalues[i] * 0.3 * math.cos(i/360 * 2 * math.pi + (math.pi))) + 360
			y = (self.lidarvalues[i] * 0.3 * math.sin(i/360 * 2 * math.pi + (math.pi))) + 360
			laserdata.append([x,y])
		return laserdata

	def visualiseThread(self, queue):
		
		pygame.init()
		self.screen = pygame.display.set_mode((720, 720))
		self.clock = pygame.time.Clock()
		running = True
		while running:
			self.screen.fill("white")
			pygame.draw.circle(self.screen, "green", (360, 360), 10)
			pygame.draw.line(self.screen, "gray", (0,0),(360,360), 2)
			pygame.draw.line(self.screen, "gray", (360,360),(720,0), 2)
			#pygame.draw.arc(self.screen, "blue", ((0,0),(720,720)), 0, math.pi)
			
			for i in range(self.startangle, self.endangle):
				x = (self.lidarvalues[i] * 0.3 * math.cos(i/360 * 2 * math.pi + (math.pi))) + 360
				y = (self.lidarvalues[i] * 0.3 * math.sin(i/360 * 2 * math.pi + (math.pi))) + 360
				pygame.draw.circle(self.screen, "red", (x, y), 2)
				

			lines = queue.get()
			self.draw_lines(lines, "blue")
   
			x = (self.lidarvalues[self.ball[0]] * 0.3 * math.cos(self.ball[0]/360 * 2 * math.pi + (math.pi))) + 360
			y = (self.lidarvalues[self.ball[0]] * 0.3 * math.sin(self.ball[0]/360 * 2 * math.pi + (math.pi))) + 360
			pygame.draw.circle(self.screen, "brown", (x, y), 10)
   
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					running = False
		
			pygame.display.flip()
			self.clock.tick(60)  
	
		pygame.quit()
 
 

if __name__ == "__main__":
	# offset is to change the starting degree
	# if place the lidar right side up, 0 is left side clockwise to 360, so flip is 0 to 360 anticlockwise
	lidar = LD19('/dev/ttyAMA3', offsetdeg = 0, flip = True) 
	# use pygame to draw on the screen from 0 to 180
	#this function is now non-blocking
	lidar.visualise(0, 180) 
	

    # use the lidar to check whether the left side or right side got more space to go
	count = 0
	while True:
		time.sleep(1)
		try:
			pass
		except KeyboardInterrupt:
			lidar.terminate()
			break
		# ~ if lidar.getReading(90) < 200:
			# ~ print(count)
			# ~ count += 1
