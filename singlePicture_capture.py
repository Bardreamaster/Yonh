# Capture a single image from the webcam when a button is pressed then save the image
# Useful for capturing a relatively small number of still frames (such as for capturing calibration images)

import cv2
from datetime import datetime

# loc = '~/PycharmProjects/testDemo/images/calib/'  # Default location
loc = '/home/beckham/yonh/'
# -*- encoding: utf-8 -*-

import numpy as np
import cv2
import os
import time
from datetime import datetime



# collect images for calibration
def CalibImage_collecting(loc,WIDTH,HEIGHT,FPS,BRIGHTNESS,CONTRAST,SATURATION,HUE,EXPOSURE):
	time_start = time.perf_counter()
	key = cv2.waitKey(1)
	#print(cv2.VideoCapture(1))
	cap = cv2.VideoCapture(2)
	cap.set(3, WIDTH)
	cap.set(4,HEIGHT)
	# cap.set(5,FPS)
	cap.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))
	# cap.set(10,BRIGHTNESS)
	# cap.set(11,CONTRAST)
	# cap.set(12,SATURATION)
	# cap.set(13,HUE)
	# cap.set(15,EXPOSURE)
	print(cap.get(3))
	print(cap.get(4))
	print(cap.get(5))
	frame_int = 1
	img_int = 1
	while True:


		check, frame = cap.read()
		# print(frame_int,time.perf_counter()-time_start)
		frame_int = frame_int+1
		cv2.imshow("Capturing", frame)
		key = cv2.waitKey(1)
		# if img_int > 100:
		# 	print(str(time.perf_counter()-time_start))
		if key == ord('s'):
			print("Capturing image...")
			img_filename = str(img_int) + '.png'
			now = datetime.now()
			if not os.path.exists(loc):
				os.makedirs(loc)
			cv2.imwrite(filename=loc + img_filename, img=frame)
			print(loc + img_filename)
			img_int += 1
			print("Calib_image saved.")



		elif key == ord('q'):
			print("Turning off camera.")
			cap.release()
			print("Camera off.")
			print("Program ended.")
			cv2.destroyAllWindows()
			break



if __name__ == "__main__":
	CalibImage_collecting(loc,1280,760,10,0,60,50,0,180)