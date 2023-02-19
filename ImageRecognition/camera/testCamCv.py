import cv2 as cv


if __name__=="__main__":

    cam = cv.VideoCapture(-1)
    
    while(True):

        flag, frame = cam.read()

        if(not flag):
            print("Frame grab error")
            continue

        cv.imshow("Frame", frame)

        if(cv.waitKey(1) == ord('q')):
            break
    
    cv.destroyAllWindows()