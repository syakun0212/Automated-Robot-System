### Folder for integration code

Integration Code. Controller for each task are under Algos - controllers.

File Name | Description | Input Topic | Output Topic
----------|-------------|-------------|-------------
hwInterface.py | Interfaces with STM32 | ```robot_cmd``` | ```robot_state```,```imu_data```
commandNew.py | Command Manual |  | ```robot_cmd```
commandAuto.py | Command Auto | ```robot_path``` | ```robot_cmd```
imageDetection.py | Image Detection Interface |  | ```img_result```,```trigger```
imageDetectionObs.py | Image Detection Interface modified for task 2 |  | ```img_result```,```trigger```
particleLocalizer.py | Particle Filter Localization | ```obstacle_data```,```startstop```,```start_data```,```particle_frame_update```,```trigger```,```img_result```,```path_progress``` | ```robot_est_pose```,```send_message```
ParticleUpdater.py | Particle Filter Odometry Update Utility | ```robot_state```,```imu_data``` | ```particle_frame_updater```
plot.py | Plot data from robot | ```robot_state```,```imu_data``` | 
BTservice.py | Bluetooth service | ```send_message```, ```Obstacl_ImageID``` | ```obstacle_data```,```start_data```,```startstop```


Task 1 Scripts:
- hwInterface.py
- BTservice.py
- ParticleUpdater.py
- particleLocalizer.py
- imageDetection.py
- controller/controllerNew.py

Task 2 Scripts:
- hwInterface.py
- BTservice.py
- ParticleUpdater.py
- imageDetectionObs.py
- controller2/Controller4.py
