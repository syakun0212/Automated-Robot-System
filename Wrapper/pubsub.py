import fastdds
import time
import RobotCore.Core
import idl_data.IMU_data.IMU_build.IMU as IMU


class PubSub(object):
    def __init__(self):
        self.core = RobotCore.Core.Core(0)

        self.pub = self.core.createPublisher(IMU, "imu_data1", 10)
        self.sub = self.core.createSubscriber(IMU, "imu_data2", self.relay, 10)

    def relay(self, data):
        print(f"Acceleration X: {data.accelerationX()}")
        print(f"Acceleration Y: {data.accelerationY()}")
        print(f"Angular Velocity Z: {data.angularVelZ()}")
        self.pub.publish(data)

if __name__ == "__main__":
    node = PubSub()

    try:
        while(True):
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exit")





    
