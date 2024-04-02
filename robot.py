import time
import serial


def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))

class Robot:
	def __init__ (self, port, baud = 115200):
		self.ser = serial.Serial(
			port=port, 
			baudrate = baud,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE,
			bytesize=serial.EIGHTBITS,
			timeout=1
		)
		
	def move(self, lspeed, rspeed):
		print("speed", lspeed, rspeed)
		s = "M " + str(clamp(int(lspeed), -255, 255)) + " " + str(clamp(int(rspeed), -255, 255)) + "\r\n"
		self.ser.write(s.encode("utf-8"))

	def light(self, R, G, B):
		s = "P " +str(clamp(int(R), 0, 255)) + " " + str(clamp(int(G), 0, 255)) + " " + str(clamp(int(B), 0, 255)) + "\r\n"
		self.ser.write(s.encode("utf-8"))

	def movedegrees(self, lspeed, rspeed, degrees):
		s = "D " + str(clamp(int(lspeed), -255, 255)) + " " + str(clamp(int(rspeed), -255, 255)) + " " + str(degrees) + "\r\n"
		self.ser.write(s.encode("utf-8"))

	def grabber(self, curr, target): # 0 up 180 down
		if target > curr:
			for i in range(curr, target,5):
				s = "G " + str(clamp(int(i), 0, 300)) + "\r\n"
				self.ser.write(s.encode("utf-8"))
				time.sleep(0.05)
		else:
			for i in range(curr, target, -10):
				s = "G " + str(clamp(int(i), 0, 300)) + "\r\n"
				self.ser.write(s.encode("utf-8"))
				time.sleep(0.05)
				
	def sorter(self, pos): # 90 to block left, 45 in middle, 0 to block right
		s = "S " + str(clamp(int(pos), 0, 300)) + "\r\n"
		self.ser.write(s.encode("utf-8"))
	
	def ramp(self, pos): # 94 is middle 55 block right 133 is block left
		s = "R " + str(clamp(int(pos), 0, 300)) + "\r\n"
		self.ser.write(s.encode("utf-8"))
		
