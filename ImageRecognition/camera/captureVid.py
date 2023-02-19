import time
from imutils.video import VideoStream
import cv2 as cv

if __name__ == "__main__":

    fps = 25
    frameInterval = 1/fps # sec delay
    outputFile = "capture1.avi"

    picam = VideoStream(usePiCamera=True).start()
    time.sleep(2.0)  # allow camera sensor to warm up

    targetSize = (640,480)

    vid = cv.VideoWriter(outputFile, cv.VideoWriter_fourcc(*'MJPG'), fps, targetSize)

    print("Starting capture")
    try:
        while True:
            image = picam.read()
            vid.write(cv.resize(image, targetSize))

            # cv.imshow("Frame", image)
            # k = cv.waitKey(1)
            # if(k == ord('q')):
            #     break

            #time.sleep(fps)

    except KeyboardInterrupt:
        pass
    print(f"Saving to {outputFile}")
    vid.release()
    cv.destroyAllWindows()
