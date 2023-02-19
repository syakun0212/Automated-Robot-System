# run this program on each RPi to send a labelled image stream
import socket
import time
from imutils.video import VideoStream
import imagezmq
from matplotlib.pyplot import yscale
from picamera import PiCamera
import cv2
import numpy as np

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

            # if(not idPub.matched()):
            #     time.sleep(0.2)
            #     continue
            
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

                bullseyeCount = 0
                xB = 0 
                yB = 0
                minXB = 0
                maxXB = 0
                minYB = 0
               
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

                    if(int(data_split[0]) == 0):
                        bullseyeCount += 1

                        xB = float(data_split[1])
                        yB = float(data_split[2])

                        minXB = int(data_split[4])
                        maxXB = int(data_split[6])
                        minYB = int(data_split[5])
                
                if(bullseyeCount < 2):
                    imgG = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
                    imgHsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
                    imgFilter = cv2.inRange(image, (0, 0, 0), (180, 255, 30))
                    _, imgB = cv2.threshold(imgG, 3, 1, cv2.THRESH_OTSU | cv2.THRESH_BINARY_INV)
                    #cv2.imshow("Frame", 255*imgB)
                    #cv2.imshow("Frame", imgFilter)
                    #cv2.waitKey(20)

                    xScale = 320/416
                    yScale = 240/416

                    upY = int(minYB * yScale)

                    # Left Box
                    rightX = int(maxXB * xScale)
                    leftBox = np.sum(imgFilter[upY:,:rightX])

                    # Right Box
                    leftX = int(minXB * xScale)
                    rightBox = np.sum(imgFilter[upY:,leftX:])

                    imgIds.append(0)
                    xmin.append(0)
                    ymin.append(0)
                    xmax.append(0)
                    ymax.append(0)
                    angle.append(0)

                    rY.append(yB)

                    print(f"LB: {leftBox} RB: {rightBox}")

                    if(leftBox > rightBox):
                        # Adds bullseye to the left
                        rX.append(xB - (0.54))

                    else:
                        # Adds bullseyes to the right
                        rX.append(xB + (0.54))

                    print(f"{imgIds} - {rX}, {rY}")

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
        
