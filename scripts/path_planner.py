#!/usr/bin/env python

import rospy, math, heapq
# Add additional imports for each of the message types used
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData, GridCells, Path
from lab4.srv import *

#Class that implements priority queue.
class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

#Class that implements grid.
class Grid:
    def __init__(self, data, height, width, resolution, origin):
        self.height = height
        self.width = width
        self.resolution = resolution
        self.origin = origin

        (x, y) = origin
        self.cellOrigin = (x + resolution/2, y + resolution/2)

        #Creates the sets for obstacles and empty cells (for faster obstacle expansion and frontier searching)
        self.obstacles = set()
        self.empty = set()

        self.data = []

        counter = 0
        previous_index = 0
        current_index = 0

        #convert 1 dimensional array to 2 dimensional array
        while current_index < len(data):
            counter += 1
            current_index = counter * width
            self.data.append(data[previous_index:current_index])
            previous_index = current_index

    #Gets the value of the cell from the grid
    def getCellValue(self, cell):
        (x, y) = cell

        return self.data[y][x]

    #Sets the value of the cell on the grid
    def setCellValue(self, cell, cellValue):
        (x, y) = cell

        self.data[y][x] = cellValue

    #Gets neighbors of the specific cell
    def getNeighbors(self, cell):
        neighborList = []

        (x, y) = cell

        for i in range(0, 3):
            for j in range(0, 3):
                neighborCell = (x - 1 + j, y - 1 + i)

                if self.isWithinGrid(neighborCell) and cell != neighborCell and self.getCellValue(neighborCell) != CellType.Obstacle:
                    neighborList.append(neighborCell)

        return neighborList

    #Checks if cell coordinate is valid
    def checkIfCellValid(self, cell):
        if cell is not self.hasProperTypeAndSize(cell):
            raise Exception("The cell should be tuple consisting of two elements!")

        if cell is not self.isWithinGrid(cell):
            raise Exception("The cell should be within grid!")

    #Checks if a given cell is a tuple that consists of 2 elements
    def hasProperTypeAndSize(self, cell):
        if cell is not tuple or len(cell) != 2:
            return False
        else:
            return True

    #Checks if a given cell is within the current grid
    def isWithinGrid(self, cell):
        (x, y) = cell

        if x >= 0 and x < self.width - 1 and y >= 0 and y < self.height:
            return True
        else:
            return False

    #Scales map to a new resolution
    def scaleMap(self, scaleFactor):
        ng_data = [] #ng stands for NewGrid

        self.obstacles.clear()
        self.empty.clear()

        if type(scaleFactor) != int:
            raise Exception("The scale factor should be an integer!")

        if scaleFactor < 1:
            raise Exception("New resolution should be larger than the old resolution!")

        ng_resolution = self.resolution * scaleFactor

        #Round up the new width and height
        ng_width = -(-self.width // scaleFactor)
        ng_height = -(-self.height // scaleFactor)

        ng_row = -1

        skip = False

        for i in range(0, self.height):
            temp_ng_row = i // scaleFactor

            #We do this check in order to make sure that we append only one row per n old cells, where n is the scaleFactor
            if ng_row != temp_ng_row:
                ng_row = temp_ng_row
                ng_data.append([])

            ng_column = -1

            for j in range(0, self.width):
                temp_ng_column = j // scaleFactor

                #We do this check in order to make sure that we append only one row per n old cells, where n is the scaleFactor
                if ng_column != temp_ng_column:
                    ng_column = temp_ng_column
                    ng_data[ng_row].append(-2) # -2 indicates that the new cell has no value assigned to it yet
                    skip = False

                if (ng_column, ng_row) in self.obstacles:
                    skip = True

                if skip:
                    continue

                currentCellValue = self.getCellValue((j, i))

                ng_oldCellValue = ng_data[ng_row][ng_column]

                if (currentCellValue == CellType.Obstacle):
                    ng_data[ng_row][ng_column] = CellType.Obstacle
                    self.obstacles.add((ng_column, ng_row))

                elif (currentCellValue == CellType.Unexplored):
                    if ng_oldCellValue != CellType.Obstacle:
                        ng_data[ng_row][ng_column] = CellType.Unexplored

                else: #empty cell
                    if ng_oldCellValue != CellType.Obstacle and ng_oldCellValue != CellType.Unexplored:
                        ng_data[ng_row][ng_column] = CellType.Empty
                        self.empty.add((ng_column, ng_row))

        self.data = ng_data
        self.height = ng_height
        self.width = ng_width
        self.resolution = ng_resolution

        (x, y) = self.origin
        self.cellOrigin = (x + ng_resolution/2, y + ng_resolution/2)

    #Expands the obstacles
    def expandObstacles(self):
        newObstacles = set()

        if not self.obstacles: #If the obstacle set is empty, then iterate through the entire map and find obstacles
            Grid.populateSetWithCells(self, self.obstacles, CellType.Obstacle)

        for obstacleCell in self.obstacles:
            neighborCells = self.getNeighbors(obstacleCell)

            for neighborCell in neighborCells:
                self.setCellValue(neighborCell, CellType.Obstacle)
                newObstacles.add(neighborCell)

        self.obstacles = self.obstacles.union(newObstacles)

    #Populates the set with cells that have the given value
    @staticmethod
    def populateSetWithCells(grid, set, value):
        for i in range(0, grid.height):
            for j in range(0, grid.width):
                cell = (j, i)
                cellType = grid.getCellValue(cell)
                if cellType == value:
                    set.add(cell)

    #Prints the grid (primarily used for debugging).
    def printToConsole(self):
        for i in range(0, self.height):
            for j in range(0, self.width):
                cell = (j, self.height - 1 - i)
                print self.getCellValue(cell),
            print " "


    @staticmethod
    def getHeuristic(currentCell, destinationCell):
        (currentX, currentY) = currentCell
        (destinationX, destinationY) = destinationCell

        return math.sqrt((currentX - destinationX) ** 2 +
                         (currentY - destinationY) ** 2)

    @staticmethod
    def getPathCost(currentCell, destinationCell):
        (currentX, currentY) = currentCell
        (destinationX, destinationY) = destinationCell

        xDiff = abs(currentX - destinationX)
        yDiff = abs(currentY - destinationY)

        if (xDiff > 1 or xDiff < 0) or (yDiff > 1 or yDiff < 0):
            raise Exception("getPathCost: The function estimates the cost only for adjacent cells!")

        if xDiff + yDiff == 2:
            return 1.4
        elif xDiff + yDiff == 1:
            return 1
        else:
            return 0

class PathFinder:
    def __init__(self, start, goal):
        self.frontier = PriorityQueue()
        self.parent = {}
        self.cost_so_far = {}
        self.expanded = set()

        self.waypoints = []
        self.path = []

        self.start = start
        self.goal = goal
        self.parent[start] = None
        self.cost_so_far[start] = 0
        self.frontier.put(start, 0)

    #Runs an iteration of A star. Returns true if the algorithm is done.
    def runAStarIteration(self, grid):
        if self.frontier.empty():
            raise Exception("Was not able to find a path to the destination!")

        current = self.frontier.get()
        #Add the current node to the list of expanded nodes
        self.expanded.add(current)

        if current == self.goal:
            return True

        for neighbor in grid.getNeighbors(current):
            new_cost = self.cost_so_far[current] + Grid.getPathCost(current, neighbor)
            if neighbor not in self.cost_so_far or new_cost < self.cost_so_far[neighbor]:
                self.cost_so_far[neighbor] = new_cost
                priority = new_cost + Grid.getHeuristic(neighbor, self.goal)
                self.frontier.put(neighbor, priority)
                self.parent[neighbor] = current

        return False

    def findPath(self):
        current = self.goal

        while current != self.start:
            self.path.append(current)
            current = self.parent[current]

        self.path.append(current)
        self.path.reverse()

    def calculateWaypoints(self):
        if len(self.path) <= 1:
            raise Exception("Error: Cannot extract waypoints from the empty path or path that consists of only one coordinate!")

        lastXDiff = self.path[1][0] - self.path[0][0]
        lastYDiff = self.path[1][1] - self.path[0][1]

        for i in range(1, len(self.path) - 1):
            xDiff = self.path[i+1][0] - self.path[i][0]
            yDiff = self.path[i+1][1] - self.path[i][1]

            if lastXDiff != xDiff or lastYDiff != yDiff:
                self.waypoints.append(self.path[i])
                lastXDiff = xDiff
                lastYDiff = yDiff

        self.waypoints.append(self.path[len(self.path) - 1])

#A class that has a function of cell type enumeration.
class CellType:
    Unexplored = -1
    Empty = 0
    Obstacle = 100

#Processes the received occupancy grid message.
def processOccupancyGrid(gridMessage):
    grid = Grid(originalGridMessage.data, gridMessage.info.height, gridMessage.info.width, gridMessage.info.resolution,
                (gridMessage.info.origin.position.x, gridMessage.info.origin.position.y))
    grid.scaleMap(4)
    grid.expandObstacles()

    return grid

#Callback function that processes the initial position received.
def convertPointToCell(point, gridOrigin, resolution):
    (gridOriginX, gridOriginY) = gridOrigin

    goalPosCellX = int((point.x - gridOriginX) // resolution)
    goalPosCellY = int((point.y - gridOriginY) // resolution)

    tempCell = (goalPosCellX, goalPosCellY)

    if not grid.isWithinGrid(tempCell):
        raise Exception("Error: The position selected was outside of the grid! Please, try again.")

    return tempCell

#Converts cells to points
def convertCellToPoint(cell, cellOrigin, resolution):
    (x, y) = cell
    (cellOriginX, cellOriginY) = cellOrigin

    point = Point()
    point.x = cellOriginX + resolution*x
    point.y = cellOriginY + resolution*y

    return point

#Publishes the grid as GridCells for RViz.
def publishGridCells(frontier, expanded):
    frontierGridCells = createGridCellsMessage()
    expandedGridCells = createGridCellsMessage()

    alreadyPrintedFrontierCells = set()

    for tuple in frontier.elements:
        cell = tuple[1]
        if cell not in expanded and cell not in alreadyPrintedFrontierCells:
            frontierGridCells.cells.append(convertCellToPoint(cell, grid.cellOrigin, grid.resolution))
            alreadyPrintedFrontierCells.add(cell)

    for cell in expanded:
        expandedGridCells.cells.append(convertCellToPoint(cell, grid.cellOrigin, grid.resolution))

    frontier_cell_pub.publish(frontierGridCells)
    expanded_cell_pub.publish(expandedGridCells)


#Creates a GridCells message and auto-initializes some fields.
def createGridCellsMessage():
    gridCells = GridCells()
    gridCells.header.seq = 0
    gridCells.header.stamp = rospy.Time.now()
    gridCells.header.frame_id = "map"
    gridCells.cell_width = grid.resolution
    gridCells.cell_height = grid.resolution

    gridCells.cells = []

    return gridCells


def publishPath(path):
    pathGridCells = createGridCellsMessage()

    for cell in path:
        pathGridCells.cells.append(convertCellToPoint(cell, grid.cellOrigin, grid.resolution))

    path_cell_pub.publish(pathGridCells)

def handleRequest(req):
    global originalGridMessage
    global grid

    originalGridMessage = req.map

    grid = processOccupancyGrid(originalGridMessage)

    startCell = convertPointToCell(req.initPos.pose.pose.position, grid.origin, grid.resolution)
    goalCell = convertPointToCell(req.goalPos.pose.position, grid.origin, grid.resolution)

    pathFinder = PathFinder(startCell, goalCell)

    if (startCell == goalCell):
        pathFinder.waypoints.append(goalCell)
    else:
        aStarDone = False

        while not aStarDone:
            aStarDone = pathFinder.runAStarIteration(grid)

        publishGridCells(pathFinder.frontier, pathFinder.expanded)

        pathFinder.findPath()

        publishPath(pathFinder.path)

        pathFinder.calculateWaypoints()

    #convert the waypoints to the trajectory offsets:
    path = Path()
    path.poses = []

    (cellOriginX, cellOriginY) = grid.cellOrigin

    for cell in pathFinder.waypoints:
        (x, y) = cell

        poseObj = PoseStamped()
        poseObj.pose.position.x = cellOriginX + x * grid.resolution
        poseObj.pose.position.y = cellOriginY + y * grid.resolution
        poseObj.pose.position.z = 0

        path.poses.append(poseObj)

    return TrajectoryResponse(path)

# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('path_planner')

    global expanded_cell_pub
    global frontier_cell_pub

    expanded_cell_pub = rospy.Publisher('/expandedGridCells', GridCells, queue_size=5) # Publisher for commanding robot motion
    frontier_cell_pub = rospy.Publisher('/frontierGridCells', GridCells, queue_size=5) # Publisher for commanding robot motion
    path_cell_pub = rospy.Publisher('/pathGridCells', GridCells, queue_size=5)

    print "Starting..."

    s = rospy.Service('calculateTrajectory', Trajectory, handleRequest)
    print "Service is active now!"
    rospy.spin()

    print "Complete!"
