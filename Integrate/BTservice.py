import bluetooth #pybluez
import time

import RobotCore.Core
import idl_data.Obstacle_data.Obstacle_build.Obstacle as Obstacle
import idl_data.Message_data.Message_build.Message as Message
import idl_data.Frame_data.Frame_build.Frame as Frame
import math

core = RobotCore.Core(0)

#create publisher
obstaclepub = core.createPublisher(Obstacle, "obstacle_data", 10)
startpub = core.createPublisher(Frame, "start_data", 10)
messagepub = core.createPublisher(Message, "startstop", 10)

# #data
Obstacle_data = Obstacle.Obstacle()
Message_data = Message.Message()
Start_data = Frame.Frame()

#list bluetooth devices
# nearby_devices = bluetooth.discover_devices(lookup_names=True)
# print("Found {} devices.".format(len(nearby_devices)))

# for addr, name in nearby_devices:
#     print("  {} - {}".format(addr, name))
#end

def main():
    while (True):
        print("Bluetooth service started")
        server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        server_sock.bind(("", bluetooth.PORT_ANY))
        server_sock.listen(1)

        port = server_sock.getsockname()[1]

        #needed on pc
        # uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
        # starttime = time.time()
        # while True:
        #     try:
        #         bluetooth.advertise_service(server_sock, "Rpi Bluetooth server", service_id=uuid,
        #                             service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
        #                             profiles=[bluetooth.SERIAL_PORT_PROFILE]
        #                             )
        #         break
        #     except:
        #         time.sleep(1)

        print("Waiting for connection on RFCOMM channel", port)

        client_sock, client_info = server_sock.accept()
        print("Accepted connection from", client_info)

        #callback function of subscriber
        def msgSub(data):
            print("sending msg to android")
            print(data.message())
            time.sleep(0.1)
            client_sock.send(data.message())

        #create subscriber
        messagesub = core.createSubscriber(Message, "send_message", msgSub, 10)


        #callback function of subscriber
        def sendObsAndroid(data):
            obsDir = list(data.obstacleDir())
            print(obsDir)
            for x in range(len(obsDir)):
                time.sleep(1)
                print("TARGET,"+str(x+1)+","+obsDir[x])
                client_sock.send("TARGET,"+str(x+1)+","+obsDir[x])

        #create subscriber
        obstaclesub = core.createSubscriber(Obstacle, "Obstacle_ImageID",sendObsAndroid, 10)

        try:
            while True:
                received = client_sock.recv(1024)
                if not received:
                    break
                receivedString = received.decode('ascii').rstrip()
                parsedString = receivedString.replace("\n", ",").strip()
                print("Received:"+ parsedString )

                #parse string received
                stringList = parsedString.split(",")

                if stringList[0] == "starting":
                    try:
                        print("starting pos")
                        Start_data.x(int(stringList[1]))
                        Start_data.y(int(stringList[2]))
                        if stringList[3] == "N":
                            print("facing north")
                            Start_data.yaw(math.pi/2)
                        elif stringList[3] == "S":
                            print("facing south")
                            Start_data.yaw(-math.pi/2)
                        elif stringList[3] == "E":
                            print("facing east")
                            Start_data.yaw(0)
                        elif stringList[3] == "W":
                            print("facing west")
                            Start_data.yaw(math.pi)
                        startpub.publish(Start_data)
                    except:
                        print("error in robot start data")
                        client_sock.send("Error in obstacle start data, try sending command again")
                # movement commands
                elif stringList[0] == "W1|":
                    print("move forward")
                elif stringList[0] == "S1|":
                    print("move back")
                elif stringList[0] == "A|":
                    print("move left")
                elif stringList[0] == "D|":
                    print("move right")

                # start robot commands
                elif stringList[0] == "Start":
                    if stringList[1]=="Speed":
                        print("call speed run")
                        Message_data.message("speed")
                        messagepub.publish(Message_data)
                    elif stringList[1]=="Image":
                        print("call image run")
                        Message_data.message("image")
                        messagepub.publish(Message_data)
                    else:
                        print("error in start command")

                # obstacle info commands
                elif stringList[0]=="Obstacle count":
                    print("parse obstacle info and pub")
                    try:
                        obsX = []
                        obsY = []
                        obsDir = []

                        print("obstacle count:" + stringList[1])
                        noObstacle=int(stringList[1])
                        for x in range(noObstacle):
                            # print("obstacle number",x)
                            # print("x pos "+stringList[4+x*5])
                            obsX.append(int(stringList[4+x*5]))
                            # print("y pos "+stringList[5+x*5])
                            obsY.append(int(stringList[5+x*5]))
                            # print("facing "+stringList[6+x*5])
                            obsDir.append(stringList[6+x*5])
                        
                        print(f"obstacle X: {obsX}") 
                        print(f"obstacle Y: {obsY}") 
                        print(f"obstacle Dir: {obsDir}") 

                        Obstacle_data.obstaclesX(obsX)
                        Obstacle_data.obstaclesY(obsY)
                        Obstacle_data.obstacleDir(obsDir)

                        print(f"Publish Obst: {obstaclepub.publish(Obstacle_data)}")
                    except:
                        print("error in obstacle command")
                        client_sock.send("Error in obstacle command try sending command again")
                    
                # no commands error
                else:
                    print("message matched no commands:"+receivedString)
                    client_sock.send("message matched no commands:"+receivedString)

        except OSError:
            pass

        print("Disconnected.")
        client_sock.close()
        server_sock.close()
        messagesub.__del__()
        obstaclesub.__del__()
        print("Waiting for new connection.")

if __name__ == "__main__":
    main()