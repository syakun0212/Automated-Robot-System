import random

class Graph:
    def __init__(self, start, end):
        self.goalCycle = 0
        self.start = start
        self.end = end
        self.sx = end[0] - start[0]
        self.sy = end[1] - start[1]

        self.nodes = [start]
        self.nodeID = {start: 0}
        self.edges = []
        self.parent = {0: None}
        self.distanceFromStart = {0: 0}
        self.path = []

    def addNode(self, node, parentID, distanceFromParent):        #Add node to the graph
        try:
            self.nodeID[node]     #Check if node exists in the graph
            newNodeID = self.nodeID[node]
        except KeyError:
            newNodeID = len(self.nodes)
            self.nodeID[node] = newNodeID
            self.nodes.append(node)
            self.parent[newNodeID] = parentID
            if parentID == 0:
                self.distanceFromStart[newNodeID] = distanceFromParent
            else:
                self.distanceFromStart[newNodeID] = self.distanceFromStart[parentID] + distanceFromParent
        return newNodeID

    def addEdge(self, id1, id2, distance = None):
        self.edges.append((id1, id2)) #id1 parent

    def randomPosition(self, goals):
        greedyBias = 0.02
        randX = random.randint(0,125)
        randY = random.randint(0,125)

        # posx = self.start[0] - (self.sx / 2.) + randX * self.sx * 2.0
        # posy = self.start[1] - (self.sy / 2.) + randY * self.sy * 2.0
        if (random.random() < (1 - greedyBias)):
            posx = randX 
            posy = randY
        else:
            return self.end
            # self.goalCycle += 1
            # return (goals[self.goalCycle % 5][0], goals[self.goalCycle % 5][1])

        return (int(posx), int(posy))
