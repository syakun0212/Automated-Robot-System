# run this program on each RPi to send a labelled image stream
import socket
import time
from imutils.video import VideoStream
import imagezmq

sender = imagezmq.ImageSender(connect_to='tcp:computer_address')
rpi_name = socket.gethostname() # send RPi hostname with each image
picam = VideoStream(usePiCamera=True).start()
time.sleep(2.0)  # allow camera sensor to warm up

while True:  # send images as stream until Ctrl-C
    image = picam.read()
    reply = sender.send_image(rpi_name, image) # rpi_name = message
    #sender.send_image_reqrep(rpi_name, image) # rpi_name = message

    reply = str(reply.decode())
    print("[INFO] The reply message from Computer is: ", reply)

    if reply == "none":
        print("Nothing is detected")
    else:
        print("Image ID: ", reply)
    
