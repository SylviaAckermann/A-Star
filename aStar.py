#!/usr/bin/env python3
from eye import *
import math


# global variables
FILENAME = "nodes.txt"


class Node:
    def __init__(self, id: int, x: int, y: int):
        self.id = id
        self.x = x
        self.y = y
        self.neighbours = []

    def addNeighbour(self, neighbour):
        self.neighbours.append(neighbour)


# my_node = Node(100, 100)
# node_2 = Node(200, 200)
# my_node.addNeighbour(node_2)

# for neighbour in my_node.neighbours:
#    print(neighbour)


class Edge:
    def __init__(self, neighbour: Node, distance: int):
        self.neighbour = neighbour
        self.distance = distance


def getRelativeGoalLocation(x, y):  # -> (int, int):
    desiredXPosition = x
    desiredYPosition = y

    currentXPosition, currentYPosition, currentAngle = VWGetPosition()

    # calculate movement required to get to new point
    sPhi = math.atan2(desiredYPosition-currentYPosition,
                      desiredXPosition-currentXPosition)*180.0/math.pi
    rot = sPhi - currentAngle
    distance = math.sqrt(pow(desiredXPosition-currentXPosition,
                         2) + pow(desiredYPosition-currentYPosition, 2))
    return int(distance), int(rot)


def driveToPoint(x, y):
    distance, rotation = getRelativeGoalLocation(x, y)
    VWTurn(rotation, 25)
    VWWait()
    VWStraight(distance, 50)
    VWWait()


def printAdjacencyMatrix():
    print('\nAdjacent Matrix: ')
    for node in nodes:
        distances = [-1]*len(nodes)
        for edge in node.neighbours:
            distances[edge.neighbour.id-1] = edge.distance

        distances[node.id-1] = 0
        output = ''
        for distance in distances:
            output += '%.2f\t' % distance
        print(output)


def heuristic(node: Node):
    # get euclidian distance
    endNode = nodes[len(nodes) - 1]
    return math.sqrt((node.x-endNode.x)**2 + (node.y-endNode.y)**2)


def readFile():
    global nodes
    with open(FILENAME, "r") as f:
        lines = f.readlines()
        nodes = [None] * len(lines)
        id = 1

        for line in lines:
            entries = [int(i) for i in line.strip().split()]
            node = nodes[id-1]
            if node is None:
                node = Node(id, entries[0], entries[1])
                nodes[id-1] = node

            neighbours = entries[2:]
            LCDCircle(2*int(128*node.x/2000), 256-2 *
                      int(128*node.y/2000), 7, WHITE, 1)

            for neighbour in neighbours:
                neighbourNode = nodes[neighbour-1]
                if neighbourNode is None:
                    neighbourEntries = [
                        int(x) for x in lines[neighbour - 1].strip().split()]
                    neighbourNode = Node(
                        neighbour, neighbourEntries[0], neighbourEntries[1])
                    nodes[neighbour-1] = neighbourNode

                distance = math.sqrt(
                    (node.x-neighbourNode.x)**2 + (node.y-neighbourNode.y)**2)
                node.addNeighbour(Edge(neighbourNode, distance))
                LCDLine(2*int(128*node.x/2000), 256-2*int(128*node.y/2000), 2 *
                        int(128*neighbourNode.x/2000), 256-2*int(128*neighbourNode.y/2000), WHITE)

            id += 1


def getPathAstar():

    startNode = nodes[0]
    endNode = nodes[len(nodes)-1]

    openSet = []
    closedSet = []
    parent = {}

    gScores = {}
    fScores = {}

    # TODO: initialise fScores, gScores and openSet

    while len(openSet) != 0:
        current = None  # TODO get node in openset with lowest f score

        if current.id == endNode.id:  # at the goal
            # Retrieve path from parents and return
            currentNode = endNode
            path = []
            previousNode = endNode
            totalDistance = 0

            while currentNode != startNode:
                # TODO add to the path
                # TODO Update total distance
                # TODO Update the current and previous node
                pass

            path.append(currentNode)  # The last node
            path.reverse()  # Put it in the right order

            print("Path distance: ", totalDistance)
            print("shortest path: ", [node.id for node in path])
            return path

        openSet.remove(current)
        closedSet.append(current)

        for edge in current.neighbours:
            neighbour = edge.neighbour

            if neighbour in closedSet:
                continue

            tentativeGScore = 0  # TODO calculate from gScore and edge distance

            # tentative path is better than recorded path
            if tentativeGScore < gScores[neighbour]:
                # TODO record parent
                # TODO record gScore and fScore using heuristic

                if neighbour not in openSet:
                    openSet.append(neighbour)

    print("No path found")
    return []

# ************************** MAIN ******************************************


if __name__ == "__main__":
    # SIMSetRobot(0, 300, 300, 100, -90)
    VWSetPosition(100, 100, 90)

    LCDMenu("Start", "A*", "Drive", "End")
    endSim = False

    while endSim == False:
        key = KEYRead()
        if (key == KEY1):
            LCDPrintf("Load File\n")
            readFile()
            printAdjacencyMatrix()
            break
        if (key == KEY2):
            LCDPrintf("Find Shortest Path\n")
            # path getPathAStar()
            break
        if (key == KEY3):
            LCDPrintf("DRIVE\n")
            # previousNode = path[0]
            # for node in path:
            #    driveToPoint(nade.x, node.y)
            #    LCDLine(2*int(128 node.x/2000), 256-2*int(128*node.y/2000), 2*int(128*previousNode.x/2000), 256-2*int(128*previousNode.y/2000))
            #    previousNode = node
            # LCDLine(2 * int(128*node.x/2000), 256-2*int(128 node.y/2000), 2*int(128*previousNode.x/2000), 256-2*int(128*previousNode.y/2000),...
            break
        if (key == KEY4):
            endSim = True
            break

    KEYWait(KEY4)
