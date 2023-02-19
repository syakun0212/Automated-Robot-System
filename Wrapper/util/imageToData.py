import py
import cv2
import idl_data.ImageType1_data.ImageType1_build.ImageType1 as ImageType


def imageToData(image):
    data = ImageType.ImageType1()
    height, width, color = image.shape
    data.height(height)
    data.width(width)
    data.color(color)
    img_size = height*width*color
    # image_1D_array = image.reshape(img_size).tolist()
    # data.imgData(image_1D_array)
    data.imgData([1,2,3,4,5])
    return data
