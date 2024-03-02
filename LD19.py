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


class LD19:
	def __init__ (self, port, baud = 230400, offsetdeg = 0, flip = False):
		self.readings = [0]* 360
		self.lidarvalues = multiprocessing.Array('i',range(360))
		self.obstacles = multiprocessing.Array('i',range(3))
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
	def visualise(self, startangle = 0, endangle = 360):
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
	
	def getObstacle(self):
		threshold = 40
		self.obstacles = []
		for i in range(self.startangle, self.endangle-1):
			if self.getReading(i+1) - self.getReading(i) > threshold:
				self.obstacles.append(i)
    
  
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
			
			for o in self.obstacles:
				x = (self.lidarvalues[o] * 0.3 * math.cos(i/360 * 2 * math.pi + (math.pi))) + 360
				y = (self.lidarvalues[o] * 0.3 * math.sin(i/360 * 2 * math.pi + (math.pi))) + 360
				pygame.draw.circle(self.screen, "blue", (x, y), 2)

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
		lidar.getObstacle()
		try:
			pass
		except KeyboardInterrupt:
			lidar.terminate()
			break
		if lidar.getReading(90) < 200:
			print(count)
			count += 1
