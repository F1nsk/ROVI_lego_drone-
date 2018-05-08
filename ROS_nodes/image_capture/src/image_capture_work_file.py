#!/usr/bin/env python

import cv2


cam = cv2.VideoCapture(0)

while True:
	ret, frame = cam.read()
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	cv2.namedWindow("feed", cv2.WINDOW_NORMAL)

	cv2.imshow("feed",gray)

cam.release()
cv2.destroyAllWindows()
