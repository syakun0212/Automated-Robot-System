import cv2 as cv


if __name__=="__main__":
    pre = input("Prefix: ")
    print(f"Saving as {pre}_count.png")
    cam = cv.VideoCapture(-1)
    
    count = 0
    while(True):

        flag, frame = cam.read()

        if(not flag):
            print("Frame grab error")
            continue

        cv.imshow(f"{pre} Frame", frame)

        k = cv.waitKey(1)
        new_symbol = "0703_symbols/"
        if(k == ord('s')):
            cv.imwrite(f"{new_symbol}{pre}_{count}.png", frame)
            count+=1
        if(k == ord('q')):
            break
    
    cv.destroyAllWindows()