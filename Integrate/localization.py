######  Useful scripts for ref: ######
## pub.py - sample code to pull the data 
## sub.py - the ref where we can create a code to pull the data 

## simplelocalize.py - pull data sample 
## plot - pull data sample 

#%% Imports
import time
import RobotCore.Core as RobotCore
import idl_data.IMU_data.IMU_build.IMU as IMU

#%% Data pull function 
def IMUDataPull(data):
    print(f"Acceleration X: {data.accelerationX()}")
    print(f"Acceleration Y: {data.accelerationY()}")
    print(f"Angular Velocity Z: {data.angularVelZ()}")

core = RobotCore.Core(0)

pub = core.createPublisher(IMU, "imu_data2",IMUDataPull, 10)

try:
    while(True):
        ## function 
        time.sleep(5)
except KeyboardInterrupt:
    print("Exiting")