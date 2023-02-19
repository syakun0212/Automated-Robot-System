import sys
sys.path.append("C:\\Users\\Edwin\\Documents\\CZ3004 - Multi Disciplinary Project")
import numpy as np
import matplotlib.pyplot as plt
from Obstacle import Obstacle

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
            print('Intersection point detected at:', intersectionPoint)
            return intersectionPoint
        else:
            print('No single intersection point detected')
            return [None, None]
    except np.linalg.LinAlgError:
        print('No single intersection point detected')
        return [None, None]

def cutsObstacle(obstacles, node, pos):
    # TODO: Check if the line cuts through any obstacles
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
            if intersectionPoint[0] != None:
                x = intersectionPoint[0]
                y = intersectionPoint[1]
                if x <= 0 and x <= 200 and y >= 0 and y <= 200:
                    return True

    return True

def main():
    obstacles = []

    # for i in range(5):
    #     x = input("X coordinate of obstacle " + str(i) + ": ")
    #     y = input("Y coordinate of obstacle " + str(i) + ": ")
    #     obstacles.append(Obstacle(int(x), int(y)))

    obstacles.append(Obstacle(160,50))
    obstacles.append(Obstacle(120,120))
    obstacles.append(Obstacle(50,40))
    obstacles.append(Obstacle(20,160))
    obstacles.append(Obstacle(70,80))

    for obstacle in obstacles:
        lineSegments = [[[obstacle.x - 20, obstacle.y - 20], [obstacle.x - 20, obstacle.y + 20]],
                        [[obstacle.x - 20, obstacle.y - 20], [obstacle.x + 20, obstacle.y - 20]],
                        [[obstacle.x + 20, obstacle.y + 20], [obstacle.x + 20, obstacle.y - 20]],
                        [[obstacle.x + 20, obstacle.y + 20], [obstacle.x - 20, obstacle.y + 20]]]
        for line in lineSegments:
            xValues = [line[0][0], line[1][0]]
            yValues = [line[0][1], line[1][1]]

            plt.plot(xValues, yValues)

    node = [20, 30]
    pos = [20, 60]
    cutsObstacle(obstacles, node, pos)

    plt.plot([node[0], pos[0]], [node[1], pos[1]])
    plt.show()



if __name__ == '__main__':
    main()
