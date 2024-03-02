import time
import serial
import threading
import pygame
import math
import multiprocessing

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
		self.linestarts = multiprocessing.Array('i',range(10))
		self.lineends = multiprocessing.Array('i',range(10))
		self.weirdpoints = multiprocessing.Array('i',range(180))
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
			self.visualisethread =  multiprocessing.Process(target = self.visualiseThread)
			self.visualisethread.start()
		else:
			print("visualisation already started")
	def stopvisualise(self):
		if self.visualisation:
			self.visualisethread.terminate()
			self.visualisation = False
 
 
	# def getObstacle(self):
	# 	threshold = 50
	# 	prevdist = 0
	# 	maxdiff = 0
	# 	maxloc = 0
	# 	for i in range(self.startangle+1, self.endangle-1):
			
	# 		b = self.lidarvalues[i]
	# 		c = self.lidarvalues[i+1]
			
			
	# 		dist = math.sqrt(b**2 + c**2 - 2*b*c*math.cos(1/180 * math.pi))
	# 		#print(i , dist)
			
	# 		if i == 1:
	# 			prevdist = dist
	# 		else:
	# 			if dist - prevdist > maxdiff:
	# 				#xy of b
	# 				x2 = (b * 0.3 * math.cos(i/360 * 2 * math.pi + (math.pi))) + 360
	# 				y2 = (b * 0.3 * math.sin(i/360 * 2 * math.pi + (math.pi))) + 360
	# 				#xy of c
	# 				x3 = (c * 0.3 * math.cos((i+1)/360 * 2 * math.pi + (math.pi))) + 360
	# 				y3 = (c * 0.3 * math.sin((i+1)/360 * 2 * math.pi + (math.pi))) + 360
					
	# 				a = self.lidarvalues[i-1]
	# 				x1 = (a * 0.3 * math.cos((i-1)/360 * 2 * math.pi + (math.pi))) + 360
	# 				y1 = (a * 0.3 * math.sin((i-1)/360 * 2 * math.pi + (math.pi))) + 360
					
	# 				if angle_between_points(x1, y1, x2, y2, x3, y3) < 140:
	# 					maxdiff = dist - prevdist
	# 					maxloc = i
					
	# 			prevdist = dist
		
	# 	self.ballLocation[0] = maxloc
	# 	print(maxloc, maxdiff)
			
	def drawLines(self):
		# linestarts -> degree of start of a line
		# lineends -> degree of end of a line
		# weirdpoints -> points that dont fit into any line
		
		countw = 0 # indexing weirdpoints array
		count = 0 # indexing lines array
		start = self.startangle
		startx = 0
		starty = 0
		for i in range(self.startangle+1, self.endangle):
			#xy of b
			b = self.lidarvalues[i]
			x2 = (b * 0.3 * math.cos(i/360 * 2 * math.pi + (math.pi))) + 360
			y2 = (b * 0.3 * math.sin(i/360 * 2 * math.pi + (math.pi))) + 360
		
			#xy of c
			c = self.lidarvalues[i+1]
			x3 = (c * 0.3 * math.cos((i+1)/360 * 2 * math.pi + (math.pi))) + 360
			y3 = (c * 0.3 * math.sin((i+1)/360 * 2 * math.pi + (math.pi))) + 360

			#xy of a
			a = self.lidarvalues[i-1]
			x1 = (a * 0.3 * math.cos((i-1)/360 * 2 * math.pi + (math.pi))) + 360
			y1 = (a * 0.3 * math.sin((i-1)/360 * 2 * math.pi + (math.pi))) + 360

			angle = angle_between_points(x1, y1, x2, y2, x3, y3)
			print(i, angle)
			
			#supposed to be if the angle is about 180 then continue the line
			if angle <= 200 and angle >= 120:
				continue
			else: # line has ended
				if i - start >= 20: # if there are more than 20 points in the line 
					self.linestarts[count] = start
					self.lineends[count] = i
					print(self.lineends[count])
					count += 1
					start = i + 1
							
				else: # short lines, points that dont belong anywhere
					self.weirdpoints[countw] = start
					countw += 1
					start = i+1
					
		print(self.weirdpoints[0])
					
		
  
	def visualiseThread(self):
		pygame.init()
		self.screen = pygame.display.set_mode((720, 720))
		self.clock = pygame.time.Clock()
		running = True
		while running:
			self.screen.fill("white")
			pygame.draw.circle(self.screen, "green", (360, 360), 10)
			pygame.draw.arc(self.screen, "blue", ((0,0),(720,720)), 0, math.pi)
			for i in range(self.startangle, self.endangle):
				x = (self.lidarvalues[i] * 0.3 * math.cos(i/360 * 2 * math.pi + (math.pi))) + 360
				y = (self.lidarvalues[i] * 0.3 * math.sin(i/360 * 2 * math.pi + (math.pi))) + 360
				pygame.draw.circle(self.screen, "red", (x, y), 2)
			
			self.drawLines()
			for i in range(len(self.linestarts)):
				if self.lineends[i] == 0:
					break
				start = self.linestarts[i]
				startx = (self.lidarvalues[start] * 0.3 * math.cos(start/360 * 2 * math.pi + (math.pi))) + 360
				starty = (self.lidarvalues[start] * 0.3 * math.sin(start/360 * 2 * math.pi + (math.pi))) + 360
				
				end = self.lineends[i]
				endx = (self.lidarvalues[end] * 0.3 * math.cos(end/360 * 2 * math.pi + (math.pi))) + 360
				endy = (self.lidarvalues[end] * 0.3 * math.sin(end/360 * 2 * math.pi + (math.pi))) + 360
				print("line at", start, end)
				pygame.draw.line(self.screen, "blueviolet", (startx, starty), (endx, endy), 3)

			# ~ for w in self.weirdpoints:
				# ~ pygame.draw.circle(self.screen, "blue", w, 4)
   
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
	lidar.visualise(10, 170) 
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
