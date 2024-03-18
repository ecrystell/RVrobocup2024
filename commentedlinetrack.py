import time  # Importing the time module for time-related operations
import cv2  # Importing OpenCV library for image processing
import numpy as np  # Importing numpy for array manipulation
import RPi.GPIO as GPIO  # Importing Raspberry Pi GPIO library
import serial  # Importing the serial module for serial communication
from robot import*  # Importing custom robot module

cap = cv2.VideoCapture(0)  # Initializing video capture from camera
resolution = (320, 240)  # Setting resolution for the camera
cap.set(3, resolution[0])  # Setting width of the frame
cap.set(4, resolution[0])  # Setting height of the frame

time.sleep(2)  # Pausing execution for 2 seconds to allow camera to initialize

x_last = resolution[0]/2  # Initializing last x-coordinate
y_last = resolution[0]/2  # Initializing last y-coordinate
invert = False  # Flag for inverting color
grayscale = False  # Flag for grayscale mode

run = False  # Flag for robot movement
time.sleep(1)  # Pausing execution for 1 second
speed = 50  # Initializing speed
kp = 1.5  # Initializing proportional gain
r = Robot('/dev/serial0')  # Initializing robot object with serial communication
Greendected = False  # Flag for green detection
x_min = 160  # Initializing minimum x-coordinate
y_min = 160  # Initializing minimum y-coordinate
uturn = False  # Flag for U-turn
turnleft = False  # Flag for left turn
turnright = False  # Flag for right turn

# Function to clamp a value within a range
def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))

# Loop to continuously process frames from the camera
while True:
    _, original = cap.read()  # Reading frame from camera
    
    image = original[0:270, 0:320]  # Cropping the frame
    Greendected = False  # Resetting green detection flag
    
    # Processing based on grayscale or HSV color space
    if grayscale:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # Converting to grayscale
        Blackline= cv2.inRange(image, (0), (50))  # Thresholding for black lines
    else:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # Converting to HSV color space
        roi = image[200:300, 0:320]  # Region of interest
        Blackline= cv2.inRange(image, (0, 0, 0), (255, 255, 60))  # Thresholding for black lines
        Greensign = cv2.inRange(roi, (20,130,70), (90,255,255))  # Thresholding for green sign
        kernel = np.ones((3,3), np.uint8)  # Initializing kernel for morphological operations
        Greensign = cv2.erode(Greensign, kernel, iterations=5)  # Eroding green sign
        Greensign = cv2.dilate(Greensign, kernel, iterations=9)  # Dilating green sign
    
    # Thresholding and processing for red line
    kernel = np.ones((3,3), np.uint8)
    Redline = cv2.inRange(roi, (160,75,115), (180,220,255))
    Redline = cv2.erode(Redline, kernel, iterations=5)
    Redline = cv2.dilate(Redline, kernel, iterations=9)
    contours_red, _ = cv2.findContours(Redline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    # Exiting loop if red line is detected
    if len(contours_red) > 0:
        r.move(0,0)
        break
    
    # Morphological operations on black line
    Blackline = cv2.erode(Blackline, kernel, iterations=6)
    Blackline = cv2.dilate(Blackline, kernel, iterations=10)
    
    # Inverting black line if needed
    if invert:
        Blackline = cv2.bitwise_not(Blackline)
    
    # Finding contours of black line
    contours_blk,hierarchy = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2:]
    contours_grn, hierarchy_grn = cv2.findContours(Greensign.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    contours_blk_len = len(contours_blk)
    
    # Processing based on green detection
    if len(contours_grn) > 0:
        # Drawing rectangle around green square
        x_grn, y_grn , w_grn, h_grn = cv2.boundingRect(contours_grn[0])
        centerx_grn = int(x_grn + (w_grn/2))
        
        # Drawing line in center of green square
        cv2.line(image, (centerx_grn, 200), (centerx_grn, 250),(0,0,255),3)
        
        # Checking if green is behind black line
        checkimage = image[100:200, x_grn:(x_grn+w_grn)]
        checkGreen = cv2.inRange(checkimage, (0,0,0), (255,255,60))
        checkGreen = cv2.erode(checkGreen, kernel, iterations=5)
        checkGreen = cv2.dilate(checkGreen, kernel, iterations=9)
        contours_chk, hierarchy_chk = cv2.findContours(checkGreen.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours_chk) > 0:
            if len(contours_grn) == 2:
                cv2.putText(image, "U-turn", (50,180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0),3)
                uturn = True
                Greendected = False
            else:
                Greendected = True
    
    # Drawing rectangle around green square
    cv2.rectangle(image, (x_grn, y_grn+250), (x_grn+w_grn, y_grn+h_grn+250), (255, 0, 0), 2)
    
    # Processing based on black line detection
    if contours_blk_len > 0:
        if contours_blk_len == 1:
            blackbox = cv2.minAreaRect(contours_blk[0])
        else:
            canditates=[]
            off_bottom = 0      
            for con_num in range(contours_blk_len):
                blackbox = cv2.minAreaRect(contours_blk[con_num])
                (x_min, y_min), (w_min, h_min), ang = blackbox
                box = cv2.boxPoints(blackbox)
                (x_box,y_box) = box[0]
                off_bottom += 1
                canditates.append((y_box,con_num,x_min,y_min))
            canditates = sorted(canditates)
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
                (y_highest,con_highest,x_min, y_min) = canditates[-1]
                blackbox = cv2.minAreaRect(contours_blk[con_highest])
        (x_min, y_min), (w_min, h_min), ang = blackbox
        setpoint = resolution[0]/2
        error = int(x_min - setpoint)
        
        # Moving the robot based on error and run flag
        if run:
            r.move(clamp(int(speed + error*kp), -255, 255) , clamp(int(speed - error * kp), -255, 255))
        else:
            r.move(0, 0) 
        ang = int(ang)    
        box = cv2.boxPoints(blackbox)
        box = np.int0(box)
        cv2.drawContours(image,[box],0,(0,0,255),3)
        cv2.putText(image,str(error),(10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.line(image, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)
    
    # Processing based on green detection results
    if Greendected:
        if centerx_grn > (x_min):
            cv2.putText(image, "Turn Right", (50,180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0),3)
            turnright = True
        elif centerx_grn < (x_min):
            cv2.putText(image, "Turn Left", (50,180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0),3)
            turnleft = True
    
    # Moving the robot based on detection results
    if run:
        if uturn:
            r.movedegrees(50,50,20)
            r.movedegrees(-50, 50, 35)
            time.sleep(3)
            uturn = False
        elif turnright:
            r.movedegrees(50,50,20)
            r.movedegrees(50, -50, 18)
            time.sleep(3)
            turnright = False
        elif turnleft:
            r.movedegrees(50,50,20)
            r.movedegrees(-50, 50, 18)
            time.sleep(3)
            turnleft = False
    
    # Displaying processed frames
    cv2.imshow("orginal", Blackline)
    cv2.imshow("orginal with line", image)
    key = cv2.waitKey(1) & 0xFF
    
    # Interacting with the code using keyboard keys
    if key == ord("q"):  # Press 'q' to exit the program
        r.move(0, 0)
        break
    elif key == ord("r"):  # Press 'r' to toggle run mode
        run = not run
    elif key == ord("e"):  # Press 'e' to increase speed by 5
        speed = speed + 5
        print("speed:" + str(speed) + "\tkp:" + str(kp))
    elif key == ord("w"):  # Press 'w' to decrease speed by 5
        speed = speed - 5
        print("speed:" + str(speed) + "\tkp:" + str(kp))
    elif key == ord("a"):  # Press 'a' to increase kp by 0.5
        kp = kp + 0.05
        print("speed:" + str(speed) + "\tkp:" + str(kp))
    elif key == ord("s"):  # Press 's' to decrease kp by 0.5
        kp = kp - 0.05
        print("speed:" + str(speed) + "\tkp:" + str(kp))
