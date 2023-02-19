
import time
import RobotCore
import idl_data.ImageType1_data.ImageType1_build.ImageType1 as ImageType
import cv2 as cv
from util import imageToData
import os

core = RobotCore.Core(0)

pub = core.createPublisher(ImageType, "image_data1", 1)
img_dir = os.getcwd()
img_filename = 'imageTest.jpg'
img = cv.imread(img_dir + '/' + img_filename)
data = imageToData(img)
print(data.imgData())
try:
    while(True):
        if(pub.publish(data)):
            print("Pub")
        time.sleep(1)
except KeyboardInterrupt:
    print("Exit")