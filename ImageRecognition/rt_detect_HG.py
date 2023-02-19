import os
import sys
from cv2 import BFMatcher_create, approxPolyDP, imshow, pointPolygonTest
import numpy as np
from datetime import datetime
import cv2 as cv
import time
import math
import pickle
import torch
import imagezmq
import glob
from sklearn.cluster import KMeans

from models.common import DetectMultiBackend
from utils.augmentations import letterbox
from utils.general import check_img_size, non_max_suppression, scale_coords, set_logging, check_imshow
from utils.torch_utils import select_device, time_sync
from utils.plots import Annotator, colors

# ******** rt_detect_HG.py is able to multiple prediction of symbols and do homograph on Obstacle symbol ********

pi_image_size1 = (640, 480)
pi_image_size2 = (480, 640)

#Image Stuff
image_size = (416, 416)
image_encoding = '.png'

# imageScale
grid_size = 10 # 1 grid on the maze is 10cm
modelScale1 = np.array([ p / m for p,m in zip(pi_image_size1, image_size) ])
modelScale2 = np.array([ p / m for p,m in zip(pi_image_size2, image_size) ])

# Sharpen filter matrix
sharpenFilter = np.array([[0, -3, 0],
                          [-3, 13, -3],
                          [0, -3, 0]])

# source obstacle symbol 4 corner coordinates(topL,topR,btmL,btmR)
src_points = np.array([[13,14], [84,14], [14,84], [84,84]], dtype=float)

line_thickness = 3
conf_thres=0.80  # 0.85 confidence threshold 
iou_thres=0.65  # NMS IOU threshold 0.45 default
results = {} #Results to store processed_image_id
device='0' # cuda device, i.e. 0 or 0,1,2,3 or cpu

image_hub = imagezmq.ImageHub()

# Location of the models
#model_path = 'C:/Users/zappe/Desktop/updated.pt'
#model_path = 'C:/Users/zappe/Desktop/2202_best.pt'
model_path = 'C:/Users/zappe/Desktop/RoboFlow Data/MDP_2202/2202_best.pt'
#model_path = 'C:/Users/zappe/Desktop/0803_best.pt' # currently the best
# model_path = 'C:/Users/zappe/Desktop/2702_best.pt'
#model_path = 'C:/Users/zappe/Desktop/RoboFlow Data/MDP_0902/5l_contrast_last.pt'

#Data YAML path
#data = 'C:/Users/zappe/Desktop/MDP Test/MDP_Final1/data.yaml'
data = 'C:/Users/zappe/Desktop/Images/MDP Test/MDP_Final1/data.yaml'

# Symbol Path
symbol_path = 'C:/Users/zappe/Desktop/yolov5/symbols'

# Save raw image path
image_path = 'C:/Users/zappe/Desktop/yolov5/capture_imgs'

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
    im_src = cv.imread(symbol_path + '/' + str(symbol_id))
    symbol_src[sym_id] = im_src
    sH, sW, _ = im_src.shape

def show_capture(path):
    path = path
    imagepath_list = [image_path for image_path in os.listdir(path)]
    counter = 0

    for index, image_path in enumerate(imagepath_list):
        if index == 0: # 0-1, 2,3, 4,5
            img1 = cv.imread(path + "/" + image_path)
            img1 = cv.resize(img1,(220,220))
            img2 = cv.imread(path + "/" + imagepath_list[index+1])
            img2 = cv.resize(img2, (220,220))
            
            Hori = np.concatenate((img1, img2), axis=1)
            
            img3 = cv.imread(path + "/" + imagepath_list[index+2])
            img3 = cv.resize(img3, (220,220))
            Hori = np.concatenate((Hori, img3), axis=1)

            cv.imwrite(f"stack_{str(counter)}.png",Hori)
            counter += 1
        if index > 1 and index%3 == 0:
            if index+2 < len(imagepath_list):
                img1 = cv.imread(path + "/" + imagepath_list[index])
                img1 = cv.resize(img1, (220,220))

                img2 = cv.imread(path + "/" + imagepath_list[index+1])
                img2 = cv.resize(img2, (220,220))
                
                Hori = np.concatenate((img1, img2), axis=1) # len(imagepath_lsit) = 4 3+2 < 4
                img3 = cv.imread(path + "/" + imagepath_list[index+2])
                img3 = cv.resize(img3, (220,220))
                Hori = np.concatenate((Hori, img3), axis=1)
                cv.imwrite(f"stack_{str(counter)}.png",Hori)
            else:
                if index+1 < len(imagepath_list):  
                    img = cv.imread(path + "/" + image_path)
                    img = cv.resize(img, (660,220))
                    cv.imwrite(f"stack_{str(counter)}.png", img)
                if index+2 <= len(imagepath_list):
                    img1 = cv.imread(path + "/" + image_path)
                    img1 = cv.resize(img, (220,220))
                    
                    img2 = cv.imread(path + "/" + imagepath_list[index+1])
                    img2 = cv.resize(img2, (220,220))
                    Hori = np.concatenate((img1, img2), axis=1)
                    Hori = cv.resize(Hori, (660,220))
                    cv.imwrite(f"stack_{str(counter)}.png",Hori)
            counter+=1

    for index in range(counter):
        if index == 0:
            img1 = cv.imread(f"stack_{str(index)}.png")
            img2 = cv.imread(f"stack_{str(index+1)}.png")
            vert = np.concatenate((img1, img2), axis=0)
        if index > 1:
            img = cv.imread(f"stack_{str(index)}.png")
            vert = np.concatenate((vert, img), axis=0)

    cv.imshow('HORIZONTAL', vert)
    cv.waitKey(0)
    cv.destroyAllWindows() 

def filterContours(contours, fillnessLimit = (0.75, 1.0)):
        filteredCont = []
        for c in contours:
            area = cv.contourArea(c)

            x,y,w,h = cv.boundingRect(c)
            
            boundingRectArea = w*h
            fillness = area / boundingRectArea
            
            aspectRatio = float(w)/h
            
            if(fillnessLimit[0] < fillness and fillness < fillnessLimit[1]):
                filteredCont.append(c)
            
        return filteredCont
    
def getSubtreeCount(hierachy, node): # node [next, prev, child, parent, index]
    count = 0
    childInd = node[2]
    if(childInd == -1):
        return 1
    child = hierachy[node[2]]
    while(True):
        count += getSubtreeCount(hierachy, hierachy[child[4]]) + 1
        if(child[0] == -1):
            break
        child = hierachy[child[0]]
    return count

def hierachySearch(hierachy):
    index = np.arange(len(hierachy[0]))
    h = np.concatenate([hierachy[0], np.expand_dims(index, -1)], 1)
    
    noParents = h[:,3] == -1
    
    noParentsIndex = index[noParents]
    
    counts = []
    
    for i in noParentsIndex:
        counts.append((i, getSubtreeCount(h, h[i])))
    
    return max(counts, key=lambda x: x[1])


def rotationMatrixToEulerAngles(R) :
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

def order_points(pts):
    
    dst_pts = pts[:,0]
    # Initialize the order in a clockwise dir (top_left, top_right, btm_left, btm_right)
    rect = np.zeros((4, 2), dtype=float)

    # the top-left point will have the smallest sum, whereas
    # the bottom-right point will have the largest sum
    s = dst_pts.sum(axis = 1)
    rect[0] = dst_pts[np.argmin(s)]
    rect[3] = dst_pts[np.argmax(s)]

    # now, compute the difference between the points, the
    # top-right point will have the smallest difference,
    # whereas the bottom-left will have the largest difference
    diff = np.diff(dst_pts, axis = 1)
    rect[1] = dst_pts[np.argmin(diff)]
    rect[2] = dst_pts[np.argmax(diff)]

    return rect 

while True: 

    rpi_name, image = image_hub.recv_image()
    
    image = cv.undistort(image, mtx, dist, None, optimal_camera_matrix)
    x, y, w, h = roi
    image = image[y:y+h, x:x+w]
    image_undistort = image.copy()
    image = cv.resize(image, imgsz)
    img_id = 0

    print('Waiting for image from RPI......', flush=True)
    processed_image_id = [] #List to store [0] id, [1] conf, [2] filepath

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
    pred = non_max_suppression(pred, conf_thres, iou_thres, max_det = 3)
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
                img_conf = "{:.2f}".format(conf.item())
                
                detected_image_path = "capture_imgs" + "/" + str(names[c]) + ".png"
                if not os.path.exists(detected_image_path):
                    if not names[c] == "00bulleye":
                        cv.imwrite(detected_image_path, annotator.result())

                # if not str(names[c]) == "00bulleye.png":
                #     detected_image_path = "capture_imgs" + "/" + str(names[c]) + ".png"
                #     if not os.path.exists(detected_image_path):
                #         cv.imwrite(detected_image_path, annotator.result())

                print('\nImage Detected: %s (Conf: %s)\n' % (str(names[c]),str(img_conf)))
            
                # append the value into the list
                processed_image_id.append((c, names[c], img_conf))
                sendId.append(names[c])
                symbol_coord.append([xmin, ymin, xmax, ymax])

    sendData = []
    for i,sId in enumerate(sendId):
        
        im_src = symbol_src[sId]
        xmin, ymin, xmax, ymax = symbol_coord[i]

        if sId == '00bulleye': # obstacle detected -> do feature mapping -> solvepnp
            # empty list for no corner detected
            dst_pts = []

            # Forming matching points for src and dst image
            matches = []

            # im_src = symbol_src[sId]

            # xmin, ymin, xmax, ymax = symbol_coord[i]

            imgCut = image[ymin:ymax, xmin:xmax].copy()

            cv.imwrite("imgG_0_far.png", imgCut)
            imgG = cv.cvtColor(imgCut, cv.COLOR_RGB2GRAY)       

            _, imgSharpBin = cv.threshold(imgG, 60, 255, cv.THRESH_OTSU)

            contours, hierachy = cv.findContours(imgSharpBin, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            
            cnts = sorted(contours, key = cv.contourArea, reverse = True)[:5]
            area = cv.contourArea(cnts[0]) # first element is the largest area
            
            print("Area: ", area)
            if area > 450:
                hull = cv.convexHull(cnts[0])
                approxErr1 = 0.02*cv.arcLength(hull, True) #epsilon to correct the error
                dst_pts = cv.approxPolyDP(hull, approxErr1, True)

            if(len(src_points) < 4 or len(dst_pts) < 4):
                continue
            else:
                dst_points = order_points(dst_pts) # Filter and order the points (topL,topR,btmL,btmR)
                dst_points_img = dst_points + np.array([[xmin,ymin]])

                src_coord1 = np.hstack([np.multiply(src_points/symScale - np.array([[0.03,0.15]]), modelScale1), np.zeros((len(src_points), 1))])
                src_coord2 = np.hstack([np.multiply(src_points/symScale - np.array([[0.03,0.15]]), modelScale2), np.zeros((len(src_points), 1))])

                _, rvec2, tvec2 = cv.solvePnP(src_coord2, dst_points_img, optimal_camera_matrix, dist)
                rotMat, jac = cv.Rodrigues(rvec2)
                eulerAngle2 = rotationMatrixToEulerAngles(rotMat)

                # Calculate the Real world X and Z (where X = x coordinates and Z = y coordinates in real world)
                _, rvec1, tvec1 = cv.solvePnP(src_coord1, dst_points_img, optimal_camera_matrix, dist)
                flat_tvec = tvec1.flatten()

                print("Real_Y before round: ", flat_tvec[2]/1.25)
                real_Y = flat_tvec[2]/1.25
                move_Z = (((flat_tvec[2]*100)/1.25)-grid_size)/grid_size
                
                print("Before rounding Real_X: ", flat_tvec[0] + 0.03 + (0.03)*(move_Z))
                dummy_X = round((flat_tvec[0] + 0.03 + (0.03)*(move_Z))*grid_size)
                real_X = flat_tvec[0] + 0.03 + (0.03)*(move_Z)

                if dummy_X == 0:
                    direction = "front"
                    real_X = 0
                elif dummy_X < 0:
                    left_offset = -0.04
                    direction = "left"
                    #real_X = round((real_X + left_offset) * grid_size) * -1
                    real_X = (real_X + left_offset)
                else:
                    right_offset = 0.03
                    direction = "right"
                    #real_X = round((real_X + right_offset) * grid_size)
                    real_X = real_X + right_offset
                
                angle = (eulerAngle2 * 180/math.pi)[1]
                print(f"Obstacle Detected:\n X-axis {real_X} block to the {direction}, Y-axis: {real_Y} block away\n")
                print(f"Angle: {angle}")
                print(f"Rotation Vector from SolvePnP: \n{eulerAngle2 * 180/math.pi} \nTranslation Vector: \n{tvec1}")

                H, status = cv.findHomography(src_points, dst_points_img) # return H (homograph matrix)src_points, dst_points
                num, Rs, Ts, Ns  = cv.decomposeHomographyMat(H, optimal_camera_matrix)
                
                for r, t in zip(Rs, Ts):
                    eulerAngle = rotationMatrixToEulerAngles(r)
                    tTransform = np.matmul(-r.T, t)
                    print(f"Rotation Vector from Decompose: \n{eulerAngle * 180/math.pi} \n \nTranslation\n {t} \n{tTransform}")

                src_key = cv.KeyPoint_convert(src_points) 
                dst_key = cv.KeyPoint_convert(dst_points) # points

                for i in range(len(src_points)):
                    match = cv.DMatch()
                    match.queryIdx = i
                    match.trainIdx = i
                    matches.append(match)

                sendStr = sId[:2] + "," + str(real_X) + "," + str(real_Y) + "," + str(int(angle)) + "," + str(int(xmin)) + "," + str(int(ymin)) + "," + str(int(xmax)) + "," + str(int(ymax))
                sendData.append(sendStr)

                kpMatchImg = cv.drawMatches(im_src.copy(),src_key,imgCut.copy(),dst_key,matches,None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

                # # Warp source image to destination based on homography
                im_out = cv.warpPerspective(im_src, H, (image.shape[0], image.shape[1]))

                # cv.imshow("Source Image", im_src)
                # cv.imshow("Destination Image", imgCut)
                # cv.imshow("WARP", im_out)
                # cv.imshow("KP Match", kpMatchImg)
        else:
            sendStr = sId[:2] + 3*",0" + "," + str(int(xmin)) + "," + str(int(ymin)) + "," + str(int(xmax)) + "," + str(int(ymax))
            sendData.append(sendStr)
        
        # Print (inference + NMS) time 
        print(f'Done: {t2 - t1:.3f}s')

    print(f'Time to process ({time.time() - t0:.3f}s)')
    
    #Messages will be sent back to RPI
    Id = [str(c) for c in sendId if c != 0]
    #sendStr = ",".join(sendId)

    print(f"Image ID Detected: {Id}")

    sendId = [str(data) for data in sendData]
    sendStr = "|".join(sendId)

    image_hub.send_reply(sendStr.encode())

    cv.imshow("Frame", annotator.result())

    if cv.waitKey(1) & 0xFF == ord('q'):
         break
  
# # After the loop release the cap object
# vid.release()
# # Destroy all the windows

cv.destroyAllWindows()