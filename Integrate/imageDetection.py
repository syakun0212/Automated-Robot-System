# run this program on each RPi to send a labelled image stream
import socket
import time
from imutils.video import VideoStream
import imagezmq
from picamera import PiCamera
import cv2

import logging

import RobotCore

# import idl_data.ImageType1_data.ImageType1_build.ImageType1 as ImageType
from idl_data.ImageLocalisation_data.ImageLocalisation_build import ImageLocalisation
from idl_data.Message_data.Message_build import Message
from util import imageToData
import os

logging.basicConfig(level=logging.DEBUG)

if __name__ == "__main__":
    core = RobotCore.Core(0)
    idPub = core.createPublisher(ImageLocalisation, "img_result", 1)
    triggerPub = core.createPublisher(Message, "trigger", 1)

    while(not idPub.matched()):
        logging.debug("Waiting for id pub match")
        time.sleep(0.5)
    
    while(not triggerPub.matched()):
        logging.debug("Waiting for trigger pub match")
        time.sleep(0.5)

    seqNum = 0

    sender = imagezmq.ImageSender(connect_to='tcp://192.168.33.22:5555')
    rpi_name = socket.gethostname() # send RPi hostname with each image
    picam = VideoStream(usePiCamera=True).start()
    time.sleep(2.0)  # allow camera sensor to warm up

    try:
        while True:  # send images as stream until Ctrl-C
            
            image = picam.read()

            triggerMsg = Message.Message()
            triggerMsg.message(f"{seqNum}")
            triggerPub.publish(triggerMsg)

            # cv2.imshow("Image", image)
            # cv2.waitKey(0)
            reply = sender.send_image(rpi_name, image) # rpi_name = message
            #sender.send_image_reqrep(rpi_name, image) # rpi_name = message

            reply = str(reply.decode())

            print("[INFO] The reply message from Computer is: ", reply)

            if len(reply) == 0:
                print("Nothing is detected")
            else:
                print("Image ID: ", reply)
                datas = reply.split("|")
                imgIds = []
                rX = []
                rY = []
                angle = []
                xmin = []
                ymin = []
                xmax = []
                ymax = []
               
                imgResult = ImageLocalisation.ImageLocalisation()
                for data in datas:
                    data_split = data.split(',')
                    imgIds.append(int(data_split[0]))
                    rX.append(float(data_split[1]))
                    rY.append(float(data_split[2]))
                    angle.append(float(data_split[3]))
                    xmin.append(int(data_split[4]))
                    ymin.append(int(data_split[5]))
                    xmax.append(int(data_split[6]))
                    ymax.append(int(data_split[7]))
                
                imgResult.sequenceNum(f"{seqNum}")
                imgResult.imageID(imgIds)
                imgResult.x(rX)
                imgResult.y(rY)
                imgResult.angle(angle)
                imgResult.xMin(xmin)
                imgResult.xMax(xmax)
                imgResult.yMin(ymin)
                imgResult.yMax(ymax)
                idPub.publish(imgResult)
            
            seqNum += 1

            time.sleep(0.02)
                
    except(KeyboardInterrupt):
        print("Exiting")
        
