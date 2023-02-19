import serial
import time
import math

if __name__=='__main__':

    ser = serial.Serial('/dev/ttyUSB0',baudrate=115200)

    wheelDataStr = "t(s),a,v,l,r\n"

    angle = 0.0 * math.pi / 180
    velocity = 0.2

    time.sleep(1)
    ser.write(f"{int(10000*angle):+07d},{int(10000*velocity):+07d}\n".encode())
    startTime = time.time()
    stopTime = 5
    stopped = False
    duration = 10
    while(True):
        try:
            if(ser.inWaiting()):
                data = ser.readline()
                if(len(data) == 0):
                    break
                try:
                    dataStr = data.decode().replace('\0','').strip()
                    a,v,l,r,x,y,yaw,w,yawD = [float(d)/10000 for d in dataStr.split(',')]

                    wheelDataStr+=f"{time.time()-startTime},{a},{v},{l},{r}\n"
                except Exception as e:
                    print("Unexpected error")
                    print(dataStr)
                    print(e)
                    
                if(duration != 0):
                    if(time.time() - startTime > duration):
                        break
                if(not stopped):
                    if(time.time() - startTime > stopTime):
                        stopped = True
                        ser.write(f"{int(10000*0):+07d},{int(10000*0):+07d}\n".encode())

        except KeyboardInterrupt:
            break

    
    ser.close()
    
    print("Writing to file")
    with open("encData.csv", 'w') as f:
        print(wheelDataStr, file=f, end='')
