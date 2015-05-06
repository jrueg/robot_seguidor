
import picamera
import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')
import cv2

camera = picamera.PiCamera()
camera.start_preview()