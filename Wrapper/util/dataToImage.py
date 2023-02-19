import cv2
import numpy as np
import ImageType


def dataToImage(data):
    nparr = np.fromstring(data.imgData(), np.uint8)
    img_np = cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_COLOR)
    img_ipl = cv2.CreateImageHeader(
        (data.width(), data.height()), cv2.IPL_DEPTH_8U, 3)
    cv2.SetData(img_ipl, img_np.tostring(),
               img_np.dtype.itemsize * 3 * data.width())
    return img_ipl
