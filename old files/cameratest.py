
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import serial
from robot import*

cap = cv2.VideoCapture(0)
resolution = (320, 240)
cap.set(3, resolution[0]) 
cap.set(4, resolution[0]) 

# ~ if serial:
	# ~ ser = serial.Serial(
			# ~ port='/dev/serial0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
			# ~ baudrate = 115200,
			# ~ parity=serial.PARITY_NONE,
			# ~ stopbits=serial.STOPBITS_ONE,
			# ~ bytesize=serial.EIGHTBITS,
			# ~ timeout=1
	# ~ )
	
# ~ print(ser)
time.sleep(2)

x_last = resolution[0]/2
y_last = resolution[0]/2
invert = False
grayscale = False
run = False
time.sleep(1)
speed = 150
kp = 0.8
r = Robot('/dev/serial0')



def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))
while True:
		
	_, original = cap.read()
	image = original
	if grayscale:
		image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		Blackline= cv2.inRange(image, (0), (50))
	else:
		Blackline= cv2.inRange(image, (0, 0, 0), (50, 50, 50))
		Greensign = cv2.inRange(image, (0,65,0), (100,200,100))
		kernel = np.ones((3,3), np.uint8)
		Greensign = cv2.erode(Greensign, kernel, iterations=5)
		Greensign = cv2.dilate(Greensign, kernel, iterations=9)
	kernel = np.ones((3,3), np.uint8)
	Blackline = cv2.erode(Blackline, kernel, iterations=5)
	Blackline = cv2.dilate(Blackline, kernel, iterations=9)
	if invert:
		Blackline = cv2.bitwise_not(Blackline) 	
	contours_blk,hierarchy = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2:]	
	contours_blk_len = len(contours_blk)
	if contours_blk_len > 0 :
		if contours_blk_len == 1 :
			blackbox = cv2.minAreaRect(contours_blk[0])
		else:
			canditates=[]
			off_bottom = 0	   
			for con_num in range(contours_blk_len):		
				blackbox = cv2.minAreaRect(contours_blk[con_num])
				(x_min, y_min), (w_min, h_min), ang = blackbox		
				box = cv2.boxPoints(blackbox)
				(x_box,y_box) = box[0]
				#if y_box > 20 :		 
				off_bottom += 1
				canditates.append((y_box,con_num,x_min,y_min))		
			canditates = sorted(canditates)
			#print(canditates)
			if off_bottom > 1:	    
				canditates_off_bottom=[]
				for con_num in range ((contours_blk_len - off_bottom), contours_blk_len):
					(y_highest,con_highest,x_min, y_min) = canditates[con_num]		
					total_distance = (abs(x_min - x_last)**2 + abs(y_min - y_last)**2)**0.5
					canditates_off_bottom.append((total_distance,con_highest))
				canditates_off_bottom = sorted(canditates_off_bottom)         
				(total_distance,con_highest) = canditates_off_bottom[0]         
				blackbox = cv2.minAreaRect(contours_blk[con_highest])	   
			else:
				print(canditates)		
				(y_highest,con_highest,x_min, y_min) = canditates[-1]		
				blackbox = cv2.minAreaRect(contours_blk[con_highest])	 
		(x_min, y_min), (w_min, h_min), ang = blackbox
		x_last = x_min
		y_last = y_min
		if ang < -45 :
			ang = 90 + ang
		if w_min < h_min and ang > 0:	  
			ang = (90-ang)*-1
		if w_min > h_min and ang < 0:
			ang = 90 + ang	  
		setpoint = resolution[0]/2
		error = int(x_min - setpoint)
		
		if run and serial:
			print("running")
			r.move(clamp(int(speed + error*kp), -255, 255) , clamp(int(speed - error * kp), -255, 255))
		else:
			if serial:
				r.move(0, 0) 
		ang = int(ang)	 
		box = cv2.boxPoints(blackbox)
		box = np.int0(box)
		cv2.drawContours(image,[box],0,(0,0,255),3)	 
		cv2.putText(image,str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
		cv2.putText(image,str(error),(10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
		cv2.line(image, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)
		
	cv2.imshow("orginal", Blackline)		
	cv2.imshow("orginal with line", image)	
	key = cv2.waitKey(1) & 0xFF	
	if key == ord("q"):
		move(0, 0)
		break
	elif key == ord("r"):
		run = not run
	elif key == ord("e"):
		speed = speed + 5
		print("speed:" + str(speed) + "\tkp:" + str(kp))
	elif key == ord("w"):
		speed = speed - 5
		print("speed:" + str(speed) + "\tkp:" + str(kp))
	elif key == ord("a"):
		kp = kp + 0.05
		print("speed:" + str(speed) + "\tkp:" + str(kp))
	elif key == ord("s"):
		kp = kp - 0.05
		print("speed:" + str(speed) + "\tkp:" + str(kp))


