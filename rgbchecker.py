import cv2
import numpy as np
import time

def do_nothing():
    pass

# create slider here
cv2.namedWindow("Slider")
cv2.resizeWindow("Slider", 640, 480)
cv2.createTrackbar("Hue Min", "Slider", 0, 255, do_nothing)
cv2.createTrackbar("Hue Max", "Slider", 0, 255, do_nothing)
cv2.createTrackbar("Saturation Min", "Slider", 0, 255, do_nothing)
cv2.createTrackbar("Saturation Max", "Slider", 0, 255, do_nothing)
cv2.createTrackbar("Value Min", "Slider", 0, 255, do_nothing)
cv2.createTrackbar("Value Max", "Slider", 0, 255, do_nothing)

cap = cv2.VideoCapture(0)
resolution = (320, 240)
cap.set(3, resolution[0]) 
cap.set(4, resolution[0]) 

time.sleep(2)


#img = cv2.imread("./images/image1.jpg")
#img = cv2.resize(img, (640, 480))


while True:
	_, img = cap.read()
	hue_min = cv2.getTrackbarPos("Hue Min", "Slider")
	hue_max = cv2.getTrackbarPos("Hue Max", "Slider")
	sat_min = cv2.getTrackbarPos("Saturation Min", "Slider")
	sat_max = cv2.getTrackbarPos("Saturation Max", "Slider")
	val_min = cv2.getTrackbarPos("Value Min", "Slider")
	val_max = cv2.getTrackbarPos("Value Max", "Slider")


    # set bounds
	lower_bound = np.array([hue_min, sat_min, val_min])
	upper_bound = np.array([hue_max, sat_max, val_max])

    # convert to HSV image
	hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


    # create mask
	mask = cv2.inRange(hsv_img, lower_bound, upper_bound)


	resulting_img = cv2.bitwise_and(img, img, mask=mask)

	stacked_imgs = np.hstack([img, resulting_img])

#     create a stacked image of the original and the HSV one.
	cv2.imshow("Image", stacked_imgs)
	cv2.imshow("Mask", mask)
	if cv2.waitKey(1) & 0xFF == ord("q"):
		break

cv2.destroyAllWindows()