import time
import RobotCore as RobotCore
import idl_data.IMU_data.IMU_build.IMU as IMU
import idl_data.Obstacle_data.Obstacle_build.Obstacle as Obstacle
import idl_data.Message_data.Message_build.Message as Message

core = RobotCore.Core(0)

messagepub = core.createPublisher(Message, "message1", 10)
Message_data = Message.Message()
Message_data.message("test message")


obstaclepub = core.createPublisher(Obstacle, "obs_data", 10)
Obstacle_data = Obstacle.Obstacle()
Obstacle_data.obstaclesX([1,2,3])
Obstacle_data.obstaclesY([3,2,1])
Obstacle_data.obstacleDir(["N","S","E"])


try:
    while(True):
        # pub.publish(data)
        messagepub.publish(Message_data)
        obstaclepub.publish(Obstacle_data)
        print("Publishing")
        time.sleep(1)
except KeyboardInterrupt:
    print("Exit")
