import time
import serial
import threading
import pygame
import math

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
        self.readings = [0] * 360
        self.port = port
        self.flip = flip
        self.offsetdeg = offsetdeg
        self.serial = serial.Serial(
            port=port,
            baudrate=baud,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        if not self.serial:
            print("unable to initialize")
        self.thread = threading.Thread(target = self.readData) # starts a separate thread by itself when run
        self.thread.start()

    def readData(self):
        while 1:
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
                    for i in range (12):
                        angle = (int((Startangle + i * step)/100) + self.offsetdeg + 360) % 360
                        if self.flip:
                            angle = int(359 - angle)
                        self.readings[angle] = min(readings[i]['reading'], 1200) # 

    def getReading(self, angle): # returns distance from the dot, one dot every degree
        return self.readings[int(angle)] # length 360 eg if want 45 degrees, js read readings[45]
    
    def visualise(self, startangle = 0, endangle = 360):
        pygame.init()
        screen = pygame.display.set_mode((720, 720))
        clock = pygame.time.Clock()
        running = True
        startangle = (startangle) % 360
        endangle = (endangle) % 360
        
        while running:
            screen.fill("white")
            pygame.draw.circle(screen, "green", (360, 360), 10)
            #for i in range (24):
            pygame.draw.arc(screen, "blue", ((0,0),(720,720)), 0, math.pi) 
            for i in range(startangle, endangle):
                x = (self.readings[i] * 0.3 * math.cos(i/360 * 2 * math.pi + (math.pi))) + 360
                y = (self.readings[i] * 0.3 * math.sin(i/360 * 2 * math.pi + (math.pi))) + 360
                pygame.draw.circle(screen, "red", (x, y), 2)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
        
            pygame.display.flip()
            clock.tick(60)  

        pygame.quit()

    def getReading(self, angle):
        return self.readings[int(angle)]
        
if __name__ == "__main__":
	# offset is to change the starting degree
    lidar = LD19('/dev/ttyAMA3', offsetdeg = 0, flip = True) # if place the lidar right side up, 0 is left side clockwise to 360, so flip is 0 to 360 anticlockwise
    lidar.visualise(0, 180) # use pygame to draw on the screen from 0 to 180
    # use the lidar to check whether the left side or right side got more space to go
