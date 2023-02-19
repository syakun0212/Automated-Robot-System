import sys
sys.path.append("C:\\Users\\Edwin\\Documents\\CZ3004 - Multi Disciplinary Project")
import numpy as np
import math
import time
import multiprocessing
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
from matplotlib.animation import FuncAnimation
from Obstacle import Obstacle
from Grid import Grid
from Graph import Graph

def intersectionSolver(p1, p2, p3, p4):
    # Line 1 dy, dx and determinant
    a11 = (p1[1] - p2[1])
    a12 = (p2[0] - p1[0])
    b1 = (p1[0]*p2[1] - p2[0]*p1[1])

    # Line 2 dy, dx and determinant
    a21 = (p3[1] - p4[1])
    a22 = (p4[0] - p3[0])
    b2 = (p3[0]*p4[1] - p4[0]*p3[1])

    # Construction of the linear system
    # coefficient matrix
    A = np.array([[a11, a12],
                  [a21, a22]])

    # right hand side vector
    b = -np.array([b1,
                   b2])

    # solve
    try:
        intersectionPoint = np.linalg.solve(A,b)
        #Check if intersection points are within the range that we should be concern about
        if (((intersectionPoint[0] >= p1[0] and intersectionPoint[0] <= p2[0]) or (intersectionPoint[0] >= p2[0] and intersectionPoint[0] <= p1[0]))
            and ((intersectionPoint[0] >= p3[0] and intersectionPoint[0] <= p4[0]) or (intersectionPoint[0] >= p4[0] and intersectionPoint[0] <= p3[0]))
            and ((intersectionPoint[1] >= p1[1] and intersectionPoint[1] <= p2[1]) or (intersectionPoint[1] >= p2[1] and intersectionPoint[1] <= p1[1]))
            and ((intersectionPoint[1] >= p3[1] and intersectionPoint[1] <= p4[1]) or (intersectionPoint[1] >= p4[1] and intersectionPoint[1] <= p3[1]))):
            # print('Intersection point detected at:', intersectionPoint)
            return intersectionPoint
        else:
            # No intersection point
            return [None, None]
    except np.linalg.LinAlgError:
        # No intersection point
        return [None, None]

def cutsObstacle(obstacles, node, pos):
    #Check if the line cuts through any obstacles
    p1 = np.asarray(node)
    p2 = np.asarray(pos)

    for obstacle in obstacles:
        lineSegments = [[[obstacle.x - 20, obstacle.y - 20], [obstacle.x - 20, obstacle.y + 20]],
                        [[obstacle.x - 20, obstacle.y - 20], [obstacle.x + 20, obstacle.y - 20]],
                        [[obstacle.x + 20, obstacle.y + 20], [obstacle.x + 20, obstacle.y - 20]],
                        [[obstacle.x + 20, obstacle.y + 20], [obstacle.x - 20, obstacle.y + 20]]]

        for lineSegment in lineSegments:
            p3 = lineSegment[0]
            p4 = lineSegment[1]

            intersectionPoint = intersectionSolver(p1, p2, p3, p4)
            # print(intersectionPoint)
            if intersectionPoint[0] != None:        #If there exists an intersection point, the line connecting the random position and node is invalid
                return True
    return False

def calcDistance(p1, p2):
    #Calculate distance between the two points
    return int(math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2))

def nearest(graph, pos, obstacles):
    #Nearest Node
    pos = np.array(pos)
    nodes = np.array(graph.nodes)
    distanceFromPos = nodes - pos
    nodeDist = np.sum(np.multiply(distanceFromPos, distanceFromPos), axis=1)
    nodeDistIndexSort = np.argsort(nodeDist)
    nodeDistSort = nodeDist[nodeDistIndexSort]
    index = 0;
    for i in nodeDistIndexSort:
        node = graph.nodes[i]
        # if tuple(pos) == graph.end:
        #     print(node)
        if cutsObstacle(obstacles, node, pos):
            continue
        index = i;
        break;

    return graph.nodes[index], index, nodeDistIndexSort, nodeDistSort

def steer(randPos, nearestNode, stepSize):
    theta = math.atan2(randPos[1] - nearestNode[1], randPos[0] - nearestNode[0])
    x = nearestNode[0] + stepSize * math.cos(theta)
    y = nearestNode[1] + stepSize * math.sin(theta)
    return (int(x), int(y))

def RRT(conn_animate, result_queue, grid, obstacles, goals, start=None, end=None):
    startTime = time.time()
    distanceFromClosestNode = float('inf')
    graph = Graph(start, end)
    i = 0

    for i in range(1500):
        # print(i)
        randPos = graph.randomPosition(goals)

        if randPos[0] < 0 or randPos[0] > 200 or randPos[1] < 0 or randPos[1] > 200 or not grid.nodes[randPos[0]][randPos[1]]:        #If random position lies in obstacles or out of range
            # print(randPos)
            continue

        nearestNode, nearestNodeID, a, b = nearest(graph, randPos, obstacles)     #Find the nearest node to this random position which path does not cut through obstacle
        if nearestNode is None or nearestNode == randPos:                         #If unable to find continue to generate another random position
            continue

        newNode = steer(randPos, nearestNode, 5)     #Create a new point in between the random position and the nearest node

        #Check for nearby nodes to the new point
        a, b,nearbyID, nearbyDist = nearest(graph, newNode, obstacles)

        #Iterate through the nearby nodes from the new point
        leastCost = float('inf')
        leastCostID = None
        distance = None

        for i in range(len(nearbyID)):
            nearbyNodeID = nearbyID[i]
            nearbyNodeDist = nearbyDist[i]

            #If out of radius stop searching
            if (nearbyNodeDist > 5 ** 2):
                break
            
            if nearbyNodeID == 0:   #nearby node is parent
                costThroughNearbyNode = 0 + int(math.sqrt(nearbyNodeDist))
            else:
                costThroughNearbyNode = graph.distanceFromStart[nearbyNodeID] + int(math.sqrt(nearbyNodeDist))

            if (costThroughNearbyNode < leastCost):
                if (cutsObstacle(obstacles, newNode, graph.nodes[nearbyNodeID])):
                    continue
                leastCost = costThroughNearbyNode
                leastCostID = nearbyNodeID
                distance = int(math.sqrt(nearbyNodeDist))

        if leastCostID == None:
            leastCostID = nearestNodeID
            distance = calcDistance(nearestNode, newNode)

        # Add edge to from the new node to leastCostNode. Set the leastCostNode as its parent
        try:
            graph.nodeID[newNode]     #Check if new node exists in the graph
            continue
        except:
            pass

        if not grid.nodes[newNode[0]][newNode[1]]:
            continue
        newNodeID = graph.addNode(newNode, leastCostID, distance)
        graph.addEdge(leastCostID, newNodeID)

        #Rewire
        for i in range(len(nearbyID)):
            nearbyNodeID = nearbyID[i]
            nearbyNodeDist = nearbyDist[i]      #Squared distance

            #If out of radius stop searching
            if (nearbyNodeDist > 5 ** 2):
                break

            #Calculate cost of the nearby node in radius of new node through the new node
            costThroughNewNode = graph.distanceFromStart[newNodeID] + int(math.sqrt(nearbyNodeDist))

            #Rewire the nearby node if the cost through the new node is lesser than present
            if (costThroughNewNode < graph.distanceFromStart[nearbyNodeID]):
                if (cutsObstacle(obstacles, newNode, graph.nodes[nearbyNodeID])):
                    continue
                graph.edges.remove((graph.parent[nearbyNodeID], nearbyNodeID))
                graph.distanceFromStart[nearbyNodeID] = costThroughNewNode
                graph.parent[nearbyNodeID] = newNodeID
                graph.addEdge(newNodeID, nearbyNodeID)

        # # If the new point is close enough to the end point we add an edge
        # distanceFromEndPoint = calcDistance(newNode, graph.end)
        # if distanceFromEndPoint < 10:
        #     endID = graph.addNode(graph.end, newNodeID, distanceFromEndPoint)
        #     graph.addEdge(newNodeID, endID)
        #     break
        # if distanceFromEndPoint < distanceFromClosestNode:
        #     distanceFromClosestNode = distanceFromEndPoint
        # if i == 1000 and distanceFromClosestNode > 10:
        #     i = 0

        # if (i % 5 == 0):
        #     conn_animate.send(graph)
    print("Time elapsed:", time.time() - startTime)
    print("Number of points generated: ", len(graph.nodes))
    conn_animate.send(graph)
    print("End of RRT function")
    return graph


def runGraph(conn_animate, obstacles, result_queue):
    time.sleep(1)
    fig, ax = plt.subplots(figsize=(7, 7))
    plt.rc('font', size=8)
    ax.set_xlim([0, 200])
    ax.set_ylim([0, 200])
    plt.style.use('fivethirtyeight')
    receive = True

    def animate(t, conn, obstacles):
        graph = None
        if conn.poll():
            graph = conn.recv()
        else:
            return
        plt.cla()

        for obstacle in obstacles:
            lineSegments = [[[obstacle.x - 20, obstacle.y - 20], [obstacle.x - 20, obstacle.y + 20]],
                            [[obstacle.x - 20, obstacle.y - 20], [obstacle.x + 20, obstacle.y - 20]],
                            [[obstacle.x + 20, obstacle.y + 20], [obstacle.x + 20, obstacle.y - 20]],
                            [[obstacle.x + 20, obstacle.y + 20], [obstacle.x - 20, obstacle.y + 20]]]

            for line in lineSegments:
                xValues = [line[0][0], line[1][0]]
                yValues = [line[0][1], line[1][1]]
                plt.plot(xValues, yValues, linestyle='dashed', color='grey', linewidth = 0.8)

            lineSegments = [[[obstacle.x - 5, obstacle.y - 5], [obstacle.x - 5, obstacle.y + 5]],
                            [[obstacle.x - 5, obstacle.y - 5], [obstacle.x + 5, obstacle.y - 5]],
                            [[obstacle.x + 5, obstacle.y + 5], [obstacle.x + 5, obstacle.y - 5]],
                            [[obstacle.x + 5, obstacle.y + 5], [obstacle.x - 5, obstacle.y + 5]]]

            for line in lineSegments:
                xValues = [line[0][0], line[1][0]]
                yValues = [line[0][1], line[1][1]]
                plt.plot(xValues, yValues, linestyle='dashed', color='red', linewidth = 1.0)


        x = [node[0] for node in graph.nodes]
        y = [node[1] for node in graph.nodes]
        plt.scatter(x, y, c='cyan', marker = "+", s=0.5)

        for edge in graph.edges:
            node1 = graph.nodes[edge[0]]
            node2 = graph.nodes[edge[1]]
            xValues = [node1[0], node2[0]]
            yValues = [node1[1], node2[1]]
            plt.plot(xValues, yValues, color='blue', linewidth = 0.5)
        
        for i in range(len(graph.path) - 1):
            node1 = graph.path[i]
            node2 = graph.path[i + 1]
            xValues = [node1[0], node2[0]]
            yValues = [node1[1], node2[1]]
            plt.plot(xValues, yValues, color='red', linewidth = 0.8)
            time.sleep(0.05)


    ani = FuncAnimation(plt.gcf(), animate, fargs=(conn_animate,obstacles), interval=50, repeat=False)
    plt.tight_layout()
    plt.show()

    print("End of runGraph Function")

def tracePath(parent_conn_animate, graph, end, obstacles):
    #Trace path from end to beginning. If end node does not exist then find the nearest node to the end to connect
    try:
        graph.nodeID[end]
    except KeyError:
        nearestNode, nearestNodeID, a, b = nearest(graph, end, obstacles)
        print(nearestNode)
        distance = calcDistance(nearestNode, end)
        endID = graph.addNode(end, nearestNodeID, distance)
        graph.addEdge(nearestNodeID, endID)

    #Print the path and return an array of the x and y coordinates
    parent = graph.parent[graph.nodeID[end]]
    path = [end]
    while parent is not None:
        path.insert(0, graph.nodes[parent])
        parent = graph.parent[parent]

    graph.path = path
    parent_conn_animate.send(graph)
    return path

def main():
    obstacles = []
    obstacles.append(Obstacle(160,50))
    obstacles.append(Obstacle(120,120))
    obstacles.append(Obstacle(50,40))
    obstacles.append(Obstacle(20,160))
    obstacles.append(Obstacle(70,80))

    goals = [[140, 50], [100, 120], [70, 40], [20, 140], [90, 80]]

    grid = Grid(obstacles)
    start = (25,25)
    end = (91, 80)

    #Queue to return graph
    result_queue = multiprocessing.Queue()
    print("Queue size: ", result_queue.qsize())

    # creating a pipe to send the graph from RRT function to animate function
    parent_conn_animate, child_conn_animate = multiprocessing.Pipe()

    # creating new processes
    p1 = multiprocessing.Process(target=runGraph, args=(child_conn_animate, obstacles, result_queue,))
    p1.start()

    graph = RRT(parent_conn_animate, result_queue, grid, obstacles, goals, start, end)
    tracePath(parent_conn_animate,graph, end, obstacles)

    # wait until processes finish
    p1.join()

    print("P1 finished")
    print("Quitting...")

if __name__ == '__main__':
    # obstacles = []
    # obstacles.append(Obstacle(160,50))
    # obstacles.append(Obstacle(120,120))
    # obstacles.append(Obstacle(50,40))
    # obstacles.append(Obstacle(20,160))
    # obstacles.append(Obstacle(70,80))
    # print(cutsObstacle(obstacles, (103,80), (90, 80)))
    main()
    
    
