import time
from imutils.video import VideoStream
import cv2 as cv

picam = VideoStream(usePiCamera=True).start()
time.sleep(2.0)  # allow camera sensor to warm up

while True:  # send images as stream until Ctrl-C
    image = picam.read()
    cv.imwrite(f"chess7.png", image)