import socket
import math

# Socket server
socketIP = "127.0.0.1"
socketPort = 8005

# Command
angle = 0 * math.pi / 180
velocity = 0.0


if __name__ == "__main__":
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((socketIP, socketPort))
    sendStr = f"{int(10000*angle):+07d},{int(10000*velocity):+07d}\n"
    sock.send(sendStr.encode())
    sock.close()