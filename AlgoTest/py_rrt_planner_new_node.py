import rclpy

from rclpy.node import Node
from std_msgs.msg import Header, String
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from cv_bridge import CvBridge
from cpp_rrt_1.msg import StartPlan

import cv2 as cv
import numpy as np

import math
import random
import time

#Utilities Function
def rotation2D(angle):
    #Angle in radians CCW
    cT = math.cos(angle)
    sT = math.sin(angle)
    return np.array([[cT, -sT], [sT, cT]])

class RRT(object):
    def __init__(self, mapImgG: np.ndarray):
        #Map in binary 'color space'
        self.reset()
        self.setMap(mapImgG)
    
    def reset(self):
        self.nodes = []
        self.idTable = dict()
        self.nodeChild = dict()
        self.nodeParent = dict()
        self.nodeCost = []
        self.path = []
        self.reach = []
        self.samples = []
        self.maskCalcs = dict()
        
        self.map = None
        self.mShape = None
        
        self.start = None
        self.end = None
        
        self.found = False
    
    def setMap(self, mapImgG: np.ndarray):
        self.map = mapImgG
        self.mShape = np.array(self.map.shape)
        
    def setGoals(self, startPoint: tuple, endPoint: tuple):
        self.start = startPoint
        self.end = endPoint
        
        self.nodes.append(self.start)
        self.nodeCost.append(0.0)
        self.nodeChild[self.start] = []
        self.nodeParent[self.start] = None
        self.idTable[self.start] = 0
        
    def roundClipToEdge(self, vertices):
        #vertices is (Nx2) array
        return np.clip(np.round(vertices).astype(np.int32), [0,0], self.mShape[::-1]-1)
    
    def steer(self, v1, v2, dQ):
        #Straight line steer from v1 to v2 
        #Give vertex
        
        moveDir = v2-v1
        moveAng = np.arctan2(moveDir[1], moveDir[0])
        pathV = np.vstack([np.arange(0, math.ceil(dQ), 1, dtype=np.float32), np.zeros(dQ)])
        pathT = np.matmul(rotation2D(moveAng), pathV)
        
        #print((pathT.T+v1).shape)
        
        pathPixels = self.roundClipToEdge(pathT.T + v1)
        
        #Filter for only unique pixels
        pathPixelsEncode = pathPixels[:,0]*self.mShape[1] + pathPixels[:,1]
        uniquePixels, uniqueIndex = np.unique(pathPixelsEncode, return_index=True)
        uniqueIndex.sort()
        
        #Unique Index contains filtered(no duplicate) sorted list of pixels from v1 to v2
        pathPixelsF = pathPixels[uniqueIndex]
        pathPixelsFChange = np.concatenate([pathPixelsF[1:]-pathPixelsF[:-1], np.zeros((1,2), dtype=np.int32)])
        
        #Check connecting cardinal pixels of Diagonal moves]
        checkFreeCardinal = np.logical_or(self.map[pathPixelsF[:,1]+pathPixelsFChange[:,1],pathPixelsF[:,0]] == 0,self.map[pathPixelsF[:,1],pathPixelsF[:,0]+pathPixelsFChange[:,0]] == 0)
        #and check if the pixel itself is free
        checkFreeBlocks = np.logical_and(self.map[pathPixelsF[:,1], pathPixelsF[:,0]] == 0, checkFreeCardinal)
        
        if(np.all(checkFreeBlocks)):
            return pathPixelsF, True
        
        #Find index of first free pixel next to v1
        #If none is free then v1 is blocked in the movement direction -> Dont add vertex cuz it will dup with v1
        hitInd = -1 if np.all(np.logical_not(checkFreeBlocks)) else np.argmin(checkFreeBlocks)
        
        #Return list of pixel up to the blocked pixel and whether it covers the dQ
        if(hitInd != -1):
            return pathPixelsF[:hitInd], False
        else:
            return pathPixelsF[:0], False
    
    def obstacleFree(self, v1, v2):
        #Check if v2 can be reached by v1 with straight line
        pathPixels, _ = self.steer(v1,v2, math.ceil(math.sqrt(np.sum(np.square(self.mShape)))))
        try:
            if(np.any(np.all(pathPixels == v2, axis=1))):
                return True
        except:
            pass
        return False
        
    def recalculateCost(self, p, v):
        #Recalculate cost of tree <parent + nested child>
        ind = self.idTable[v]
        #self.nodeCost[ind] = (v[0]-p[0])**2+(v[1]-p[1])**2 + self.nodeCost[self.idTable[p]] #Cost calculated with distance squared
        self.nodeCost[ind] = math.sqrt((v[0]-p[0])**2+(v[1]-p[1])**2) + self.nodeCost[self.idTable[p]]
        for c in self.nodeChild[v]:
            self.recalculateCost(v, c)
        
    
    def addNode(self, dQ, greedyBias=0.05, strict=False, neighbourRad = 0):
        #dQ > 0
        #Random Config
        while True:
            
            if(random.random() < (1-greedyBias)):
                rConf = self.roundClipToEdge(np.multiply(self.mShape[::-1], np.random.uniform(size=(2))))
            else:
                rConf = np.array(self.end)
            
            if(self.map[rConf[1], rConf[0]] == 0):
                break
        
        self.samples.append(tuple(rConf))
        
        #Nearest Node
        nodeVec = np.array(self.nodes, dtype=np.float32)
        nodeDiff = nodeVec-rConf
        nodeDists = np.sum(np.multiply(nodeDiff, nodeDiff), axis=1)
        nodeDistsIndexSort = np.argsort(nodeDists)
        nodeDistsSort = nodeDists[nodeDistsIndexSort]
        
        extendInd = nodeDistsIndexSort[0]
        
        nearestNode = self.nodes[extendInd]
        nearestNodeNp = nodeVec[extendInd]
        
        path, maxDistMoved = self.steer(nearestNodeNp, rConf, dQ)
        
        if(len(path) == 0):
            #Blocked
            return self.found
        
        if(strict and not maxDistMoved):
            #Strictly max step no collision
            return self.found
        
        newConf = rConf if np.any(np.all(path == np.array(rConf), axis=1)) else path[-1]
        newConf = np.array(self.end) if np.any(np.all(path == np.array(self.end), axis=1)) else newConf #Forced end point if pass end point for optimization
        newConfT = tuple(newConf)

        #newConfCost = nodeDistsSort[0] + self.nodeCost[extendInd] #Cost calculated with distance squared
        newConfCost = np.sqrt(nodeDistsSort[0]) + self.nodeCost[extendInd]
        
        #Find least cost neighbour
        newConfV = nodeVec - newConf
        newConfDists = np.sum(np.multiply(newConfV, newConfV), axis=1) 
        
        
        neighbourMask = newConfDists <= neighbourRad*neighbourRad
        nodeCosts = np.array(self.nodeCost)
        #nodeAddCosts = nodeCosts + np.sqrt(newConfDists) #Cost calculate with distance squared
        nodeAddCosts = nodeCosts + np.sqrt(newConfDists)
        
        nodeAddCostsN = nodeAddCosts[neighbourMask]
        nodeAddCostsNIndexSort = np.argsort(nodeAddCostsN)
        
        indexFull = np.arange(len(self.nodes), dtype=np.int32)
        
        indexLink = indexFull[neighbourMask][nodeAddCostsNIndexSort]
        nodeAddCostsNSort = nodeAddCostsN[nodeAddCostsNIndexSort]
        costsMask = nodeAddCostsNSort < newConfCost
        
        passCosts = nodeAddCostsNSort[costsMask]
        passCostsIndex = indexLink[costsMask]
        
        minInd = extendInd
        minCost = newConfCost
        
        for i in range(len(passCostsIndex)):
            if(self.obstacleFree(nodeVec[passCostsIndex[i]], newConf)):
                minInd = passCostsIndex[i]
                minCost = passCosts[i]
                break
                
        minNode = self.nodes[minInd]

        #Flag to see if new node is inserted or not
        inserted = 0
        newPath = False
        
        addNode = False
        rewire1 = False
        rewire2 = False
        
        #Create connection to new node
        if(newConfT not in self.nodes):
            addNode = True
            if(np.all(newConf - rConf < 0.5)):
                self.reach.append(newConfT)
            
            self.idTable[newConfT] = len(self.nodes)
            self.nodes.append(newConfT)
            
            self.nodeChild[newConfT] = []
            self.nodeParent[newConfT] = minNode

            self.nodeChild[minNode].append(newConfT)
            self.nodeCost.append(minCost)
            inserted = -1
            
            if(newConfT == self.end):
                self.found = True
                newPath = True
            
        else:
            
            newConfTIndex = self.idTable[newConfT]
            
            if(self.nodeCost[newConfTIndex] > minCost):
                #Rewire to less cost parent
                rewire1 = True
                p = self.nodeParent[newConfT]
                self.nodeChild[p].remove(newConfT)

                self.nodeParent[newConfT] = minNode
                self.nodeChild[minNode].append(newConfT)

                self.recalculateCost(minNode, newConfT)

                
        #Rewire remaining neighbour nodes
        #costFromNew = minCost + newConfDists # Cost calculate with distance squared
        costFromNew = minCost + np.sqrt(newConfDists)
        newNeighbourMask = neighbourMask[:]
        newNeighbourMask[minInd] = False
        
        rewireMask = costFromNew[newNeighbourMask] < nodeCosts[newNeighbourMask]
        rewireIndex = indexFull[newNeighbourMask][rewireMask]
        
        for i in range(len(rewireIndex)):
            testNode = nodeVec[rewireIndex[i]]
            testNodeT = tuple(testNode)
            if(self.obstacleFree(testNode, newConf) and testNodeT != newConfT):
                #Rewire
                rewire2 = True
                p = self.nodeParent[testNodeT]
                self.nodeChild[p].remove(testNodeT)
                
                self.nodeParent[testNodeT] = newConfT
                self.nodeChild[newConfT].append(testNodeT)
                
                self.recalculateCost(newConfT, testNodeT)
        
        return self.found
    
    def constructPath(self):
        self.path = []
        if(self.found):
            tempN = self.end
            while(tempN is not None):
                self.path.append(tempN)
                tempN = self.nodeParent[tempN]
    
    
    def showState(self, pointerRad = 5, lineWidth = 2, path=True, reach=True, sample=True):
        #Return map marked
        img = 255*np.stack([self.map, self.map, self.map], axis=-1)
        
        #Edges
        for k,v in self.nodeParent.items():
            if(v is None):
                continue
            img = cv.line(img, k, v, (0,255,255), lineWidth)
            
        #Nodes
        nodes = self.nodes[:]
        if(self.end not in self.nodes):
            nodes.append(self.end)

        for n in nodes:
            c = (0, 255, 255)
            img = cv.circle(img, n, pointerRad, c, thickness=-1)
            
        if(len(self.path) > 0 and path):
            for i in range(len(self.path)-1):
                img = cv.line(img, self.path[i], self.path[i+1], (0,0,255), lineWidth)
            for n in self.path:
                img = cv.circle(img, n, pointerRad, (0,0,255), thickness=-1)
        
        if(reach):
            for n in self.reach:
                img = cv.circle(img, n, pointerRad, (255, 0 ,255), thickness=-1)
        
        if(len(self.samples) > 0 and sample):
            img = cv.circle(img, self.samples[-1], pointerRad, (255, 255 ,0), thickness=-1)
            
        img = cv.circle(img, self.start, pointerRad, (0, 255, 0), thickness=-1)
        img = cv.circle(img, self.end, pointerRad, (255, 0, 0), thickness=-1)
            
        return img

class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")
        self.cv_bridge = CvBridge()
        self.map = None
        self.mapW = None
        self.mapH = None
        self.start = None
        self.goal = None
        self.rrt = None
        self.stepSize = None
        self.nodeCount = None
        self.goalBias = None

        self.visualizationPublisher = self.create_publisher(Image, "/rrt_WIP", 5)
        self.pathPublisher = self.create_publisher(Path, "/path", 5)

        self.mapSubscription = self.create_subscription(OccupancyGrid, "/map", self.get_map, 5)
        self.startSubscription = self.create_subscription(Point, "/start_point", self.get_start, 5)
        self.goalSubscription = self.create_subscription(Point, "/goal_point", self.get_goal, 5)

        self.startSignalSubscription = self.create_subscription(StartPlan, "/start_plan", self.start_search, 5)
    
    def get_map(self, data):
        #nav_msg.msg.OccupancyGrid
    
        self.mapW = data.info.width
        self.mapH = data.info.height

        self.map = (np.array(data.data, dtype=np.uint8).reshape(self.mapH, self.mapW) / 100).astype(np.uint8)
        
        self.get_logger().info(f"Received Map - {self.mapW} x {self.mapH}")
        #cv.imshow("Image", self.map)

    def get_start(self, data):
        #geometry_msg.msg.Point
        self.start = (int(data.x), int(data.y))
        self.get_logger().info(f"Received Starting Point - {self.start} - {data.x} , {data.y}")
    
    def get_goal(self, data):
        #geometry_msg.msg.Point
        self.goal = (int(data.x), int(data.y))
        self.get_logger().info(f"Received Goal Point - {self.goal}")

    def publishImage(self, img):
        self.get_logger().info("Publishing WIP")
        imgMsg = self.cv_bridge.cv2_to_imgmsg(img, encoding="rgb8")

        #Set Headers
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map" #For rviz
        imgMsg.header = header

        self.visualizationPublisher.publish(imgMsg)
    
    def publishPath(self, path):
        pathMsg = Path()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map" #For rviz

        poses = []

        for po in path[::-1]:
            p = PoseStamped()

            p_point = Point(x = float(po[0])*0.01, y = -float(po[1])*0.01, z = 0.0)
            p_quat = Quaternion(x = 1.0, y = 0.0, z = 0.0, w = 0.0)

            p_pose = Pose(position=p_point, orientation=p_quat)
            p = PoseStamped(header=header,pose=p_pose)
 
            poses.append(p)
        
        pathMsg.header = header
        pathMsg.poses = poses

        self.get_logger().info("Publishing Path")
        self.pathPublisher.publish(pathMsg)

    def start_search(self, data):
        self.get_logger().info("Calling Search")
        stepSize = data.step_size
        maxCount = data.max_node_limit
        goalBias = data.goal_bias
        iterationLimit = data.max_iteration_limit
        strict = data.strict
        stopOnFound = data.stop_on_found
        neighbourRad = data.neighbour_radius
        visualizeFrequency = data.visualize_frequency

        path = data.path
        reach = data.reach
        sample = data.sample

        self.get_logger().info(f"Search Parameter - Step Size: {stepSize} Goal Bias: {goalBias} Max Node Count: {maxCount} \
                                Max Iteration Count: {iterationLimit} Strict: {strict} Neighbour Radius: {neighbourRad} \
                                Visualize Frequency: {visualizeFrequency} | Display Patth: {path} Reach: {reach} Last Sample Point: {sample}")

        self.searchRRT(stepSize=stepSize, maxCount=maxCount, goalBias=goalBias, iterationLimit=iterationLimit, strict=strict, stopOnFound=stopOnFound, neighbourRad=neighbourRad, visualizeFrequency=visualizeFrequency, path=path, reach=reach, sample=sample)
    
    def searchRRT(self, stepSize=20, maxCount=5000, goalBias=0.05, iterationLimit=100000, strict=True, stopOnFound=True, neighbourRad=0, visualizeFrequency=0, path=True, reach=False, sample=False):
        #Errors
        if(self.map is None):
            self.get_logger().info("Map not found")
            return None

        if(self.start is None):
            self.get_logger().info("Start point not specified")
            return None

        if(self.goal is None):
            self.get_logger().info("Goal point not specified")
            return None
            
        if(self.start[0] < 0 or self.start[0] > self.mapW or self.start[1] < 0 or self.start[1] > self.mapH or self.map[self.start[1], self.start[0]] != 0):
            self.get_logger().info("Invalid starting point")
            return None
            
        if(self.goal[0] < 0 or self.goal[0] > self.mapW or self.goal[1] < 0 or self.goal[1] > self.mapH or self.map[self.goal[1], self.goal[0]] != 0):
                self.get_logger().info("Invalid goal point")
                return None

        self.get_logger().info(f"Starting search: Start - {self.start} Goal - {self.goal}")

        self.rrt = RRT(self.map)
        self.rrt.setGoals(self.start, self.goal)
        
        step = stepSize
        nodeLimit = maxCount

        iterationCount = 0
        found = False
        while(len(self.rrt.nodes) < nodeLimit):
            found = self.rrt.addNode(step, greedyBias=goalBias, strict=strict, neighbourRad=neighbourRad)

            if(visualizeFrequency != 0 and (((iterationCount+1) % visualizeFrequency) == 0)):
                self.get_logger().info(f"Publishing Progress Iteration: {iterationCount+1} - Node Count: {len(self.rrt.nodes)}")
                self.rrt.constructPath()
                state = self.rrt.showState(path=path, reach=reach, sample=sample)
                self.publishImage(state)
                #self.get_logger().info(f"State: {state}")

            if(found and stopOnFound):
                self.get_logger().info(f"Goal Found on iteration {iterationCount+1}")
                
                break

            if(iterationCount > iterationLimit):
                self.get_logger().info("Iteration Limit Hits. Probably some error because this shoudln't happen.")
                break

            iterationCount += 1

        if(found):
            #Publish Path
            self.get_logger().info(f"Publishing Found Image - Node Count: {len(self.rrt.nodes)}")
            self.rrt.constructPath()
            state = self.rrt.showState(path=path, reach=reach, sample=sample)
            self.publishImage(state)

            self.get_logger().info("Publishing Path")
            self.publishPath(self.rrt.path)
        else:
            self.get_logger().info("Path not found!")


def main(args=None):
    rclpy.init(args=args)

    planner_node = PlannerNode()

    rclpy.spin(planner_node)
    planner_node.destroy_now()
    rclpy.shutdown()

if __name__ == '__main__':
    main()