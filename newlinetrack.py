import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import serial
from robot import *
from LD19 import LD19

cap = cv2.VideoCapture(0)
resolution = (320, 240)
cap.set(3, resolution[0])
cap.set(4, resolution[0])

time.sleep(2)

x_last = resolution[0] / 2
y_last = resolution[0] / 2
invert = False
grayscale = False
run = False
time.sleep(1)
speed = 50
kp = 1.5 # was 0.8
r = Robot('/dev/serial0')
lidar = LD19('/dev/ttyAMA3', offsetdeg = 0, flip = True) #offsetddeg was -90
Greendected = False
x_min = 160
y_min = 160
uturn = False
turnleft = False
turnright = False
error = 0


def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))

def checkObstacle():
    obsThreshold = 40
    minDeg = min(lidar.lidarvalues[80:100]) 
    if minDeg < obsThreshold:
        r.move(0,0)
        print('obstacle detected')
        leftAvg = sum(lidar.lidarvalues[35:55]) / 20
        rightAvg = sum(lidar.lidarvalues[125:145]) / 20
        if leftAvg > rightAvg:
            # uh hard code turn left
            pass
        else:
            # hard code pls
            pass
            
    
        


while True:
    #checkObstacle()
    _, original = cap.read()

    image = original
    Greendected = False
    if grayscale:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        Blackline = cv2.inRange(image, (0), (55))
    else:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        bottom = image[220:320, 0:320]
        top = image[120:220, 0:320]

        bottomBlack = cv2.inRange(bottom, (0, 0, 0), (255, 255, 70))
        topBlack = cv2.inRange(top, (0, 0, 0), (255, 255, 70))
        topGreen = cv2.inRange(top, (10, 110, 85), (70, 180, 240))

        kernel = np.ones((3, 3), np.uint8)  # to get the RGB thingies
        topGreen = cv2.erode(topGreen, kernel, iterations=5)  # eroding and dilating
        topGreen = cv2.dilate(topGreen, kernel, iterations=9)

        Redline = cv2.inRange(top, (160, 75, 115), (180, 220, 255))
        Redline = cv2.erode(Redline, kernel, iterations=5)  # eroding and dilating
        Redline = cv2.dilate(Redline, kernel, iterations=9)
        contours_red, _ = cv2.findContours(Redline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours_red) > 0:  # see red line stop n break
            r.move(0, 0)
            run = False

    kernel = np.ones((3, 3), np.uint8)
    topBlack = cv2.erode(topBlack, kernel, iterations=5)
    topBlack = cv2.dilate(topBlack, kernel, iterations=9)
    bottomBlack = cv2.erode(bottomBlack, kernel, iterations=5)
    bottomBlack = cv2.dilate(bottomBlack, kernel, iterations=9)

    if invert:
        Blackline = cv2.bitwise_not(Blackline)
    contours_blkTop, _ = cv2.findContours(topBlack.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_blkBtm, _ = cv2.findContours(bottomBlack.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_grn, _ = cv2.findContours(topGreen.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours_blkTop) > 0 or len(contours_blkBtm) > 0:
        errors = []
        if len(contours_blkTop) == 0:
            print('top nothing')
            contoursToCheck = [contours_blkBtm, ]
            x, y, w, h = cv2.boundingRect(contours_blkBtm[0])
            centerx = int(x + (w//2))
            cv2.line(image, (centerx, 250), (centerx, 300), (255,0,0), 3)
            setpoint = resolution[0] / 2
            error = int(x - setpoint)
            print(error)
            errors.append(error)
        elif len(contours_blkBtm) == 0:
            print('bottom nothing')
            contoursToCheck = [contours_blkTop, ]
            x, y, w, h = cv2.boundingRect(contours_blkTop[0])
            centerx = int(x + (w//2))
            cv2.line(image, (centerx, 150), (centerx, 200), (255,0,0), 3)
            setpoint = resolution[0] / 2
            error = int(x - setpoint)
            print(error)
            errors.append(error)
        else:
            print('both something!')
            contoursToCheck = [contours_blkTop, contours_blkBtm]
            x, y, w, h = cv2.boundingRect(contours_blkTop[0])
            centerx = int(x + (w//2))
            cv2.line(image, (centerx, 150), (centerx, 200), (255,0,0), 3)
            setpoint = resolution[0] / 2
            error = int(x - setpoint)
            print(error)
            errors.append(error)
            
            x, y, w, h = cv2.boundingRect(contours_blkBtm[0])
            centerx = int(x + (w//2))
            cv2.line(image, (centerx, 250), (centerx, 300), (255,0,0), 3)
            setpoint = resolution[0] / 2
            error = int(x - setpoint)
            print(error)
            errors.append(error)

        
        
        # ~ for contours in contoursToCheck:
            
            # ~ contours_blk_len = len(contours)
            # ~ if contours_blk_len == 1:
                # ~ blackbox = cv2.minAreaRect(contours[0])
            # ~ else:
                
                # ~ canditates = []
                # ~ off_bottom = 0
                # ~ for con_num in range(contours_blk_len):
                    # ~ blackbox = cv2.minAreaRect(contours[con_num])
                    # ~ (x_min, y_min), (w_min, h_min), ang = blackbox
                    # ~ box = cv2.boxPoints(blackbox)
                    # ~ (x_box, y_box) = box[0]
                    # ~ # if y_box > 20 :
                    # ~ off_bottom += 1
                    # ~ canditates.append((y_box, con_num, x_min, y_min))
                # ~ canditates = sorted(canditates)
                # ~ # print(canditates)
                # ~ if off_bottom > 1:
                    # ~ canditates_off_bottom = []
                    # ~ for con_num in range((contours_blk_len - off_bottom), contours_blk_len):
                        # ~ (y_highest, con_highest, x_min, y_min) = canditates[con_num]
                        # ~ total_distance = (abs(x_min - x_last) ** 2 + abs(y_min - y_last) ** 2) ** 0.5
                        # ~ canditates_off_bottom.append((total_distance, con_highest))
                    # ~ canditates_off_bottom = sorted(canditates_off_bottom)
                    # ~ (total_distance, con_highest) = canditates_off_bottom[0]
                    # ~ blackbox = cv2.minAreaRect(contours[con_highest])
                # ~ else:
                    # ~ print(canditates)
                    # ~ (y_highest, con_highest, x_min, y_min) = canditates[-1]
                    # ~ blackbox = cv2.minAreaRect(contours[con_highest])
                # ~ (x_min, y_min), (w_min, h_min), ang = blackbox
                # ~ # x_last = x_min
                # y_last = y_min
                # if ang < -45 :
                # ang = 90 + ang
                # if w_min < h_min and ang > 0:
                # ang = (90-ang)*-1
                # if w_min > h_min and ang < 0:
                # ang = 90 + ang
            # ~ setpoint = resolution[0] / 2
            # ~ error = int(x - setpoint)
            # ~ print(error)
            # ~ errors.append(error)

                # ~ ang = int(ang)
                # ~ box = cv2.boxPoints(blackbox)
                # ~ box = np.int0(box)
                # ~ cv2.drawContours(image, [box], 0, (0, 0, 255), 3)

                # ~ # cv2.putText(image,str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                # ~ cv2.line(image, (int(x_min), 200), (int(x_min), 250), (255, 0, 0), 3)

        if len(errors) > 1:
            if len(errors) == 1:
                error = errors[0]
            else:
                if abs(errors[0]) > abs(errors[1]):
                    error = errors[1]
                else:
                    error = errors[0]
        else:
            error = 0

        if run:
            print("running")
            r.move(clamp(int(speed + error * kp), -255, 255), clamp(int(speed - error * kp), -255, 255))
            cv2.putText(image, str(error), (10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        else:
            r.move(0, 0)
    else:
        print('where line')
    if len(contours_grn) > 0:

        # drawing rect around the green square
        x_grn, y_grn, w_grn, h_grn = cv2.boundingRect(contours_grn[0])
        centerx_grn = int(x_grn + (w_grn / 2))
        # drawing line in center of green square
        cv2.line(image, (centerx_grn, 200), (centerx_grn, 250), (0, 0, 255), 3)

        if h_grn >= 90:
            error = 0
        else:
            # check if green is behind black line
            checkimage = image[70:120, x_grn:(x_grn + w_grn)]
            checkGreen = cv2.inRange(checkimage, (0, 0, 0), (255, 255, 60))
            checkGreen = cv2.erode(checkGreen, kernel, iterations=5)
            checkGreen = cv2.dilate(checkGreen, kernel, iterations=9)
            contours_chk, hierarchy_chk = cv2.findContours(checkGreen.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours_chk) > 0:
                if len(contours_grn) == 2:
                    x_grn1, _, _, _ = cv2.boundingRect(contours_grn[1])
                    if x_grn - 10 < x_grn1 < x_grn + 10:
                        cv2.putText(image, "U-turn", (50, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
                        uturn = True
                        Greendected = False
                    else:
                        Greendected = True
                else:
                    Greendected = True

            cv2.rectangle(image, (x_grn, y_grn + 120), (x_grn + w_grn, y_grn + h_grn + 120), (255, 0, 0), 2)

    if Greendected:

        if centerx_grn > (x_min):
            cv2.putText(image, "Turn Right", (50, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
            turnright = True
        elif centerx_grn < (x_min):
            cv2.putText(image, "Turn Left", (50, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
            turnleft = True
        if run:
            if uturn:
                r.movedegrees(50, 50, 25)
                r.movedegrees(-50, 50, 35)
                r.movedegrees(-50,-50, 15)
                time.sleep(3)
                uturn = False
                
            elif turnright:
                r.movedegrees(50, 50, 25)
                r.movedegrees(50, -50, 18)
                r.movedegrees(-50,-50, 15)
                time.sleep(3)
                turnright = False
            elif turnleft:
                r.movedegrees(50, 50, 25)
                r.movedegrees(-50, 50, 18)
                r.movedegrees(-50,-50, 15)
                time.sleep(3)
                turnleft = False


    cv2.imshow("top", topBlack)
    cv2.imshow("btotom", bottomBlack)
    cv2.imshow("orginal with line", image)
    key = cv2.waitKey(1) & 0xFF

    # keys to interact with code
    if key == ord("q"):  # q: exit program
        r.move(0, 0)
        break
    elif key == ord("r"):  # r: toggle run mode
        run = not run
    elif key == ord("e"):  # e: increase speed by 5
        speed = speed + 5
        print("speed:" + str(speed) + "\tkp:" + str(kp))
    elif key == ord("w"):  # w: decrease speed by 5
        speed = speed - 5
        print("speed:" + str(speed) + "\tkp:" + str(kp))
    elif key == ord("a"):  # a: increase kp ny 0.5
        kp = kp + 0.05
        print("speed:" + str(speed) + "\tkp:" + str(kp))
    elif key == ord("s"):  # s: decrease kp by 0.5
        kp = kp - 0.05
        print("speed:" + str(speed) + "\tkp:" + str(kp))

