import serial

if __name__=='__main__':

    ser = serial.Serial('/dev/ttyUSB0',baudrate=115200)

    while(True):
        data = ser.readline()
        if(len(data) == 0):
            break
        print(data.decode())

    ser.close()