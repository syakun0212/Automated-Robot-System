import numpy as np
import cv2 as cv
import glob
import pickle
import yaml

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

frame_size = (416, 416)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('new_chess_image/chess_*.png') # make it become a list

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7,7), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (7,7), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)
cv.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, frame_size, None, None)

print("Camera calibrated: ", ret)
print("\nCamera Matrix: \n", mtx)
print("\nDistortion Parameter: \n", dist)
print("\nRotation Vectors:\n", rvecs)
print("\nTranslation Vectors:\n", tvecs)

img = cv.imread('new_chess_image/calib.png')
cv.imshow("frame", img)
cv.waitKey(0)
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

print("\nNew Camera Matrix: \n", newcameramtx)

# Save the camera calibration results into pickle
calib_result_pickle = {}
calib_result_pickle["mtx"] = mtx
calib_result_pickle["optimal_camera_matrix"] = newcameramtx
calib_result_pickle["dist"] = dist
calib_result_pickle["roi"] = roi
pickle.dump(calib_result_pickle, open("new_camera_calib_pickle.p", "wb" ))

# data = {'camera_matrix': np.asarray(mtx).tolist(),
#         'dist_coeff': np.asarray(dist).tolist(),
#         'new_camera_matrix': np.asarray(newcameramtx).tolist()} 

# with open("calibration_matrix.yaml", "w") as f:
#     yaml.dump(data, f)

"""
Next time, we just need to loadin the data using pickle again
calib_result_pickle = pickle.load(open("camera_calib_pickle.p", "rb" ))
mtx = calib_result_pickle["mtx"]
optimal_camera_matrix = calib_result_pickle["optimal_camera_matrix"]
dist = calib_result_pickle["dist"]

undistorted_image = cv2.undistort(distorted_image, mtx, dist, None, 
                                    optimal_camera_matrix)
"""

# undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imwrite('calibresult1.png', dst)

# undistort
'''
mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
print(mapx, mapy)
dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imwrite('calibresult2.png', dst)
'''

# TO see the error %
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error
print( "total error: {}".format(mean_error/len(objpoints)))

"""
Homograph Matrix - more corners better

# Load symbol path
symbol = ...

for symbol in symbols:
    if symbol == 'detected_symbol': # if detected image/obstacle do homograph
        pts_src = np.float32(pts_src) # points of the first image (xmin1, ymin1, xmax1, ymax1)
        pts_dst = np.float32(pts_dst) # points of the second images (xmin2, ymin2, xmax2, ymax2)
        H, status = cv2.findHomography(pts_src, pts_dst) # return H (homograph matrix)

        # Decompose the Matrix

        num - get num_possible solution
        Rs - contain a list of rotation matrix
        Ts - contain a list of translation vector list of the normal vector of the plane

        num, Rs, Ts, Ns  = cv2.decomposeHomographyMat(H, K)
"""