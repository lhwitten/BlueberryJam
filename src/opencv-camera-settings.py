import cv2
import numpy as np


#Example -2 (bright) -11(dark)
exposure=-5

#Example -130 (dark) +130(bright)
brightness=0

#Example -130 (dark) +130(bright)
contrast=0

#Example 0 - 500
focus=0

#0 to N (camera index, 0 is the default OS main camera)
camera_id=0

live_feed=False

vid = cv2.VideoCapture(camera_id)
if not vid.isOpened():
	raise ValueError('Unable to open video source')
blank_image = np.zeros((200,200,3), np.uint8)
print("Press the following key (lowercase or caps-lock) to change the setting:")
print("1,2,3: Switch to another webcam")
print("c/C  : decrease/increase Constrast")
print("b/B  : decrease/increase Brightness")
print("f/F  : decrease/increase Focus")
print("e/E  : decrease/increase Exposure")
print("l/L  : hide/show live stream")
print(" s   : open DirectShow settings")
print(" q   : exit the application")
while(True):
	if live_feed:
		_, frame = vid.read()
		if frame is not None:
			cv2.imshow('image',frame)
	else:
		cv2.imshow('image',blank_image)
		frame = None
		
	key = cv2.waitKey(10)
	if key == ord('q') or key == ord('Q'):
		break
	if key == ord('s') or key == ord('S'):
		print("Open DirectShow settings")
		vid.release()
		vid2 = cv2.VideoCapture(camera_id + cv2.CAP_DSHOW)
		vid2.set(cv2.CAP_PROP_SETTINGS, 1)
		vid2.release()
		vid = cv2.VideoCapture(camera_id)
	if key == ord('l'):
		print(f'hide live video camera')
		vid.release()
		vid = cv2.VideoCapture(camera_id)
		live_feed=False
	if key == ord('L'):
		print(f'show live video camera (blocked for other processes)')
		if vid.isOpened():
			vid.release()
		vid = cv2.VideoCapture(camera_id)
		live_feed=True
	if key == ord('E'):
		exposure+=0.5
		r=vid.set(cv2.CAP_PROP_EXPOSURE, exposure)
		print(f'exposure: {exposure}')
	if key == ord('e'):
		exposure-=0.5
		print(f'exposure: {exposure}')
		r=vid.set(cv2.CAP_PROP_EXPOSURE, exposure)
	if key == ord('B'):
		brightness+=10
		print(f'brightness: {brightness}')
		r=vid.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
	if key == ord('b'):
		brightness-=10
		print(f'brightness: {brightness}')
		vid.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
	if key == ord('C'):
		contrast+=10
		print(f'contrast: {contrast}')
		vid.set(cv2.CAP_PROP_CONTRAST, contrast)
	if key == ord('c'):
		contrast-=10
		print(f'contrast: {contrast}')
		vid.set(cv2.CAP_PROP_CONTRAST, contrast)
	if key == ord('F'):
		focus+=5
		print(f'focus: {focus}')
		vid.set(cv2.CAP_PROP_AUTOFOCUS, 0)
		vid.set(cv2.CAP_PROP_FOCUS, focus)
	if key == ord('f'):
		focus-=5
		print(f'focus: {focus}')
		vid.set(cv2.CAP_PROP_AUTOFOCUS, 0)
		vid.set(cv2.CAP_PROP_FOCUS, focus)
	if key >= ord('0') and key <= ord('3'):
		vid.release()
		camera_id=key-ord('0')
		vid = cv2.VideoCapture(camera_id)
		if not vid.isOpened():
			raise ValueError('Unable to open video source')
		
		
if vid.isOpened():
	vid.release()
cv2.destroyAllWindows()
