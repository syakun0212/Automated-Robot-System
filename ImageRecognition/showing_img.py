import os
import cv2 as cv
import numpy as np

path = "C:/Users/zappe/Desktop/yolov5/capture_imgs"
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
            if index+1 <= len(imagepath_list):  
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