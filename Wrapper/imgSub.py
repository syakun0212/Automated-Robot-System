import RobotCore.Core
import idl_data.ImageType_data.ImageType_build.ImageType as ImageType
import fastdds
import time


def ImageReader(data):
    print("test")
    print(f"Image Height: {data.height()}")
    print(f"Image Width: {data.width()}")
    print(f"Image Byte String: {data.imgData()}")


core = RobotCore.Core.Core(0)

sub = core.createSubscriber(ImageType, "image_data1", ImageReader, 10)

try:
    while(True):
        time.sleep(5)
except KeyboardInterrupt:
    print("Exiting")
