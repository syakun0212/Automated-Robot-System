import serial
import queue
import logging
import time
import math

import RobotCore
from idl_data.RobotState_data.RobotState_build import RobotState
from idl_data.IMU_data.IMU_build import IMU

logging.basicConfig(level=logging.DEBUG)

ROBOT_T = 0.16
ROBOT_L = 0.14
WHEEL_R = 0.0325

class LPF(object):
    def __init__(self, length, initVal):
        self.length = length
        self.buffer = [initVal] * length
        self.sum = initVal * length
        self.ind = length-1
    
    def process(self, val):
        self.sum -= self.buffer[self.ind]
        self.sum += val
        self.buffer[self.ind] = val
        self.ind -= 1
        if(self.ind < 0):
            self.ind = self.length-1
        return self.sum / self.length


class HWInterfaceNode(object):
    def __init__(self, serialPort: str, baudrate: int = 112500, restTime: float = 0.002):
        self.serialPort = serialPort
        self.baudrate = baudrate
        self.restTime = restTime

        self.core = RobotCore.Core()
        
        self.commandQueue = queue.Queue(maxsize=10)
        
        self.statePub = self.core.createPublisher(RobotState, "robot_state",1)
        self.imuPub = self.core.createPublisher(IMU, "imu_data", 1)
        self.cmdSub = self.core.createSubscriber(RobotState, "robot_cmd",  self.queueCommand, 1)

        self.connectSerialLoop()

    def queueCommand(self, data) -> None:
        
        angle = data.steer_angle()
        velocity = data.velocity()

        if(self.commandQueue.full()):
            try:
                self.commandQueue.get(block=False)
            except queue.Empty:
                logging.debug("Queue Empty")

        self.commandQueue.put(self.encodeCommand(angle, velocity))

    def encodeCommand(self, angle: float, velocity: float) -> str:
        return f"{int(10000*angle):+07d},{int(10000*velocity):+07d}\n"

    def connectSerial(self, serialPort: str, baudrate: int, connectSleep: float = 0.01) -> bool:
        self.ser = serial.Serial(serialPort, baudrate=baudrate)
        self.ser.flushInput()
        self.ser.flushOutput()
        time.sleep(connectSleep)

        if(self.ser.isOpen()):
            return True
        else:
            self.ser = None
            return False

    def connectSerialLoop(self, retryDelay: float = 5):
        while not self.connectSerial(self.serialPort, self.baudrate):
            logging.debug(f"Failed to connect serial port {self.serialPort} - {self.baudrate}, retrying in {retryDelay} seconds")
            time.sleep(retryDelay)

    def loop(self):
        try:
            
            sAngleLPF = LPF(3, 0.0)

            while(True):

                try:
                    writeString = self.commandQueue.get(block=False)
                    self.ser.write(writeString.encode())
                    logging.debug(f"Writing {writeString}")
                except queue.Empty:
                    pass
                except serial.SerialException:
                    self.ser = None
                    self.connectSerialLoop()

                if(self.ser.inWaiting() > 0):
                    dataRaw = self.ser.readline()

                    if(len(dataRaw) == 0):
                        # Disconnected
                        self.ser = None
                        self.connectSerialLoop()
                        continue

                    dataString = dataRaw.decode().replace('\0','').strip()
                    angle, cmdVel, vl, vr, accX, accY, gyroW, yaw, dYaw = [ float(t)/10000 for t in dataString.split(',') ]
                    gyroW *= 10 # Different Scaling
                    gyroW *= math.pi/180

                    imuData = IMU.IMU()
                    imuData.accelerationX(accY * 9.81)
                    #imuData.accelerationY(0.0)
                    imuData.accelerationY(-accX * 9.81)
                    imuData.angularVelZ(dYaw)

                    if(not self.imuPub.publish(imuData)):
                        # Can use to track errors
                        pass

                    stateData = RobotState.RobotState()
                    #stateData.steer_angle(angle)
                    
                    v = (-vl + vr)/2
                    '''
                    angle = (vr - (-vl)) / ROBOT_T
                    newAng = sAngleLPF.process(angle)
                    if(abs(newAng) < 1e-5):
                        newAng
                    stateData.steer_angle(newAng)
                    '''
                    stateData.steer_angle(yaw)
                    stateData.velocity(v) #vl spins negativ
                    
                    if(not self.statePub.publish(stateData)):
                        # Can use to track errors
                        pass

                time.sleep(self.restTime)

        except KeyboardInterrupt:
            logging.info("Exiting")

def main():
    node = HWInterfaceNode('/dev/ttyUSB0')
    node.loop()

if __name__ == "__main__":
    main()