# run this program on each RPi to send a labelled image stream
import socket
import time
from imutils.video import VideoStream
import imagezmq
from picamera import PiCamera
import cv2


sender = imagezmq.ImageSender(connect_to='tcp://192.168.33.22:5555')
rpi_name = socket.gethostname() # send RPi hostname with each image
picam = VideoStream(usePiCamera=True).start()
time.sleep(2.0)  # allow camera sensor to warm up

while True:  # send images as stream until Ctrl-C
    image = picam.read()

    cv2.imshow("Image", image)
    #cv2.waitKey(0)
    reply = sender.send_image(rpi_name, image) # rpi_name = message
    #sender.send_image_reqrep(rpi_name, image) # rpi_name = message

    reply = str(reply.decode())
    print("[INFO] The reply message from Computer is: ", reply)

    if reply == "none":
        print("Nothing is detected")
    else:
        print("Image ID: ", reply)
    

    
