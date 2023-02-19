import os
import sys
from cv2 import BFMatcher_create, pointPolygonTest
import numpy as np
from datetime import datetime
import cv2
import time
import math
import pickle
import torch
import imagezmq
import glob

from models.common import DetectMultiBackend
from utils.augmentations import letterbox
from utils.general import check_img_size, non_max_suppression, scale_coords, set_logging, check_imshow
from utils.torch_utils import select_device, time_sync
from utils.plots import Annotator, colors

#
pi_image_size = (640, 480)

#Image Stuff
image_size = (416, 416)
image_encoding = '.png'

line_thickness = 2
conf_thres=0.85  # confidence threshold # default 0.7
# iou_thres=0.45  # NMS IOU threshold
iou_thres=0.65
results = {} #Results to store processed_image_id
device='0' # cuda device, i.e. 0 or 0,1,2,3 or cpu

image_hub = imagezmq.ImageHub()

modelScale = np.array([ p / m for p,m in zip(pi_image_size, image_size) ])

# Location of the models
model_path = 'C:/Users/zappe/Desktop/RoboFlow Data/MDP_2202/2202_best.pt'

#Data YAML path
data = 'C:/Users/zappe/Desktop/Images/MDP Test/MDP_Final1/data.yaml'

# Symbol Path
symbol_path = 'C:/Users/zappe/Desktop/yolov5/symbols'

 # Initialize
set_logging()
device = select_device(device)

# Load model
print('[Setting Up Image Processing Server] Loading model...')
model = DetectMultiBackend(model_path, device=device, dnn=False, data=data)

stride, names = model.stride, model.names

imgsz = check_img_size(image_size, s=stride)
model.warmup(imgsz=(1, 3, *imgsz))

print('\n*************Started Image Processing Server*************\n')

calib_result_pickle = pickle.load(open("new_camera_calib_pickle.p", "rb" ))
mtx = calib_result_pickle["mtx"]
optimal_camera_matrix = calib_result_pickle["optimal_camera_matrix"]
dist = calib_result_pickle["dist"]
roi = calib_result_pickle["roi"]

#13 px to 8 mm
symScale = 13/0.008 #pix/m

symbol_coord = []
symbol_src = dict()
symbol_sift = dict()

for symbol_id in os.listdir(symbol_path):
    sym_id = symbol_id.removesuffix('.png')
    im_src = cv2.imread(symbol_path + '/' + str(symbol_id))
    symbol_src[sym_id] = im_src
    sH, sW, _ = im_src.shape

while True: 
    #ret, image = vid.read()
    rpi_name, image = image_hub.recv_image()

    image = cv2.undistort(image, mtx, dist, None, optimal_camera_matrix)
    x, y, w, h = roi
    image = image[y:y+h, x:x+w]
    image = cv2.resize(image, imgsz)
    img_id = 0

    #image = cv2.convertScaleAbs(image, 1.0, 0)
    
    print('Waiting for image from RPI......', flush=True)

    # Padded resize
    img = letterbox(image, imgsz, stride=stride, auto=True)[0]

    # Convert
    img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
    img = np.ascontiguousarray(img)

    t0 = time.time()
    
    img = torch.from_numpy(img).to(device)
    img = img.float()
    img /= 255.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    # Inference
    t1 = time_sync()
    pred = model(img)
    # NMS
    pred = non_max_suppression(pred, conf_thres, iou_thres, max_det = 5)
    t2 = time_sync()

    sendId = []
    symbol_coord = []

    # Process detections
    for i, det in enumerate(pred):  # detections per image
        gn = torch.tensor(image.shape)[[1, 0, 1, 0]] # normalization gain whwh
        annotator = Annotator(image.copy(), line_width=line_thickness, example=str(names))
        if len(det) != 0:
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], image.shape).round()

            print(det)

            # Write results
            for d in reversed(det):
                xmin, ymin, xmax, ymax, conf, cls = d.cpu().numpy()
                xmin = int(xmin)
                ymin = int(ymin)
                xmax = int(xmax)
                ymax = int(ymax)
                xyxy = [xmin, ymin, xmax, ymax]

                # Add bbox to image
                c = int(cls)
                #xmin, ymin, xmax, ymax = xyxy[0].cpu(), xyxy[1].cpu(), xyxy[2].cpu(), xyxy[3].cpu()
                print("xmin: {}, ymin: {}, xmax: {}, ymax: {}".format(xmin, ymin, xmax, ymax))

                label = f'{names[c]} {conf:.2f}'
                annotator.box_label(xyxy, label, color=colors(c, True))

                detected_image_path = "capture_imgs" + "/" + str(names[c]) + ".png"
                if not os.path.exists(detected_image_path):
                    cv2.imwrite(detected_image_path, annotator.result())

                img_conf = "{:.2f}".format(conf.item())
                print('\nImage Detected: %s (Conf: %s)\n' % (str(names[c]),str(img_conf)))
                
                # append the value into the list
                sendId.append(names[c])
                symbol_coord.append([xmin, ymin, xmax, ymax])

    for i,sId in enumerate(sendId):
        
        # Forming matching points for src and dst image
        matches = []

        im_src = symbol_src[sId]

        xmin, ymin, xmax, ymax = symbol_coord[i]

        imgCut = image[ymin:ymax, xmin:xmax].copy()

        # Print (inference + NMS) time 
        print(f'Done: {t2 - t1:.3f}s')

    print(f'Time to process ({time.time() - t0:.3f}s)')
    
    #Messages will be sent back to RPI
    sendId = [str(c[:2]) for c in sendId if c != 0]
    sendStr = ",".join(sendId)

    print(f"Image Detected: {sendStr}")

    image_hub.send_reply(sendStr.encode())

    cv2.imshow("Frame", annotator.result())

    if cv2.waitKey(1) & 0xFF == ord('q'):
         break
  
# # After the loop release the cap object
# vid.release()
# # Destroy all the windows
cv2.destroyAllWindows()

  