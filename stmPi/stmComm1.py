import serial
import time
import socket
import sys
import select

# Socket
socketIP = "127.0.0.1"
socketPort = 8005
socketClientBufferSize = 16

# STM
serPort = '/dev/ttyUSB0'
serBaudrate = 115200

if __name__ == "__main__":
    
    # Set up serial
    try:
        ser = serial.Serial(port=serPort, baudrate=serBaudrate)
    except serial.SerialException as e:
        print("Opening serial failed")
        print(e)
        sys.exit(1)

    # Set up socket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind((socketIP, socketPort))
    except socket.error as e:
        print("Creating server failed")
        print(e)

        ser.close()
        sys.exit(1)

    sock.listen(1)
    readable = [sock]
    try:
        while(True):

            r,w,e = select.select(readable, [], [], 0.001)
            for rs in r:
                if(rs == sock):
                    #server
                    conn, addr = sock.accept()
                    print(f"{addr} - Connected")
                    readable.append(rs)
                    pass
                else:
                    
                    data = rs.recv(socketClientBufferSize)

                    if(len(data) == 0):
                        readable.remove(rs)

                    try:
                        if(data is str):
                            data = data.encode()
                        ser.write(data)
                    except serial.SerialException as e:
                        print("Failed to write to serial")
                        print(e)
            
            if(ser.in_waiting > 0):
                serData = ser.readline()
                
                # Send data back to socket
                #for r in readable:
                #    if(r != sock):
                #        r.send(serData)

                serDataStr = serData.decode().replace('\0','').strip()
                l,r,x,y,w = [float(d)/10000 for d in serDataStr.split(',')]
                print(f"{l} {r} {x} {y} {w}")


    except KeyboardInterrupt:
        print("Exiting")
        pass

    #except Exception as e:
    #    print("Unexpected error")
    #    print(e)

    ser.close()
    sock.close()


