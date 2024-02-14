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
		s = "M " + str(clamp(int(lspeed), -255, 255)) + " " + str(clamp(int(rspeed), -255, 255)) + "\r\n"
		self.ser.write(s.encode("utf-8"))
	def light(self, R, G, B):
		s = "P " +str(clamp(int(R), 0, 255)) + " " + str(clamp(int(G), 0, 255)) + " " + str(clamp(int(B), 0, 255)) + "\r\n"
		self.ser.write(s.encode("utf-8"))
	def movedegrees(self, lspeed, rspeed, degrees):
		s = "D " + str(clamp(int(lspeed), -255, 255)) + " " + str(clamp(int(rspeed), -255, 255)) + " " + str(degrees) + "\r\n"
		self.ser.write(s.encode("utf-8"))
