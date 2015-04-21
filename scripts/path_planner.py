#!/usr/bin/env python

import rospy, math, heapq, Queue, threading
# Add additional imports for each of the message types used
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData, GridCells, Path
from final_project.srv import *

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

        #priority queue heap can contain duplicates
        if current not in self.expanded:
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

def processCostGrid(gridMessage, result_queue):
    #TODO: add the implementation of this function
    #return the result using the following: result_queue.put(result)
    pass

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

def publishGridCells(publisher, gridCellList):
    gridCellsMessage = createGridCellsMessage()

    for cell in gridCellList:
        gridCellsMessage.cells.append(convertCellToPoint(cell, grid.cellOrigin, grid.resolution))

    publisher.publish(gridCellsMessage)

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

#TODO: possibly create a new class that will contain this service method
def getWaypoints(req):
    startCell = convertPointToCell(req.initPos.pose.pose.position, grid.origin, grid.resolution)
    goalCell = convertPointToCell(req.goalPos.pose.position, grid.origin, grid.resolution)

    pathFinder = PathFinder(startCell, goalCell)

    if (startCell == goalCell):
        pathFinder.waypoints.append(goalCell)
    else:
        aStarDone = False

        while not aStarDone:
            aStarDone = pathFinder.runAStarIteration(grid)

        publishGridCells(expanded_cell_pub, pathFinder.expanded)

        #Convert frontier queue to the frontier set
        frontierCells = set()

        for tuple in pathFinder.frontier.elements:
            cell = tuple[1]
            if cell not in pathFinder.expanded and cell not in frontierCells:
                frontierCells.add(cell)

        publishGridCells(frontier_cell_pub, frontierCells)

        pathFinder.findPath()

        publishGridCells(path_cell_pub, pathFinder.path)

        pathFinder.calculateWaypoints()

    #convert the waypoints to the trajectory offsets:
    waypoints = Path()
    waypoints.poses = []

    (cellOriginX, cellOriginY) = grid.cellOrigin

    for cell in pathFinder.waypoints:
        (x, y) = cell

        poseObj = PoseStamped()
        poseObj.pose.position.x = cellOriginX + x * grid.resolution
        poseObj.pose.position.y = cellOriginY + y * grid.resolution
        poseObj.pose.position.z = 0

        waypoints.poses.append(poseObj)

#Gets the centroid of the largest frontier
def getCentroid(result_queue):
    visited = set()
    clusters = []

    for cell in grid.empty:
        cluster = []

        expandCluster(cell, cluster, visited)

        if len(cluster) != 0:
            clusters.append(cluster)

    if len(clusters) == 0:
        return
    else:
        #Find the largest cluster in the list of clusters
        (largestClusterIndex, largestCluster) = max(enumerate(clusters), key = lambda tup: len(tup[1]))

        print largestClusterIndex

        centroid = calculateCentroid(largestCluster)

    clusterCells = []
    for cluster in clusters:
        clusterCells += cluster

    publishGridCells(cluster_cell_pub, clusters[1])
    publishGridCells(centroid_cell_pub, [centroid])

    centroidPos = Point()
    centroidPos.x = centroid[0] + grid.cellOrigin[0]
    centroidPos.y = centroid[1] + grid.cellOrigin[1]
    result_queue.put(centroidPos)

#Adds the cell to the cluster if the cell is on the border with the unexplored cells and is not visited
def expandCluster(cell, cluster, visited):
    if cell in visited:
        return

    visited.add(cell)
    possibleClusterCandidates = []

    neighbors = grid.getNeighbors(cell)

    doesCellBelongToCluster = False

    (x, y) = cell
    for neighbor in neighbors:
        neighborValue = grid.getCellValue(neighbor)
        (neighborX, neighborY) = neighbor

        if neighborValue == CellType.Unexplored and (neighborX - x == 0 or neighborY - y == 0):
            doesCellBelongToCluster = True
        elif neighborValue == CellType.Empty:
            possibleClusterCandidates.append(neighbor)

    if doesCellBelongToCluster:
        #1) Add the cell to the cluster
        cluster.append(cell)
        #2) Check if the neighbor empty cells belong to this cluster as well
        for clusterCandidate in possibleClusterCandidates:
            expandCluster(clusterCandidate, cluster, visited)

def calculateCentroid(cluster):
    centroidX = 0
    centroidY = 0

    for cell in cluster:
        (x, y) = cell
        centroidX += x
        centroidY += y

    clusterSize = len(cluster)

    centroidX = int(round(centroidX / clusterSize))
    centroidY = int(round(centroidY / clusterSize))

    return (centroidX, centroidY)

def handleRequest(req):
    global originalGridMessage
    global grid

    print "Received request."

    originalGridMessage = req.map

    #Start the thread for the centroid finding logic
    # if req.processCostMap:
    #     result_queue = Queue.Queue()
    #     thread1 = threading.Thread(
    #             target=processCostGrid,
    #             name="ProcessCostGrid() Thread",
    #             args=[req.costMap, result_queue],
    #             )
    #     thread1.start()

    grid = processOccupancyGrid(originalGridMessage)

    # if req.processCostMap:
    #     thread1.join()
    #     #costGrid = result_queue.get()
    #     #grid.addCostGrid(costGrid) -- TODO: Add similar method to the grid class

    #By default centroid is not found
    foundCentroid = False

    #Start the thread for the centroid finding logic
    # if req.returnCentroid:
    #     result_queue = Queue.Queue()
    #     thread2 = threading.Thread(
    #             target=getCentroid,
    #             name="GetCentroid() Thread",
    #             args=[result_queue],
    #             )
    #     thread2.start()

    #Meanwhile fine the path using A star in the main thread
    waypoints = getWaypoints(req)

    result_queue = Queue.Queue()

    getCentroid(result_queue)

    centroid = Point()

    #Wait for the thread to be done and get the result
    # if req.returnCentroid:
    #     thread2.join()
    #
    #     if not result_queue.empty():
    #         centroid = result_queue.get()
    #         foundCentroid = True
    #     else:
    #         centroid = Point()
    #         foundCentroid = False

    print "Done with the request processing!"

    return TrajectoryResponse(waypoints, foundCentroid, centroid)

# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('path_planner')

    global expanded_cell_pub
    global frontier_cell_pub
    global path_cell_pub
    global cluster_cell_pub
    global centroid_cell_pub

    expanded_cell_pub = rospy.Publisher('/expandedGridCells', GridCells, queue_size=5) # Publisher for commanding robot motion
    frontier_cell_pub = rospy.Publisher('/frontierGridCells', GridCells, queue_size=5) # Publisher for commanding robot motion
    path_cell_pub = rospy.Publisher('/pathGridCells', GridCells, queue_size=5)
    cluster_cell_pub = rospy.Publisher('/clusterGridCells', GridCells, queue_size=5)
    centroid_cell_pub = rospy.Publisher('/centroidGridCell', GridCells, queue_size=5)

    print "Starting..."

    s = rospy.Service('calculateTrajectory', Trajectory, handleRequest)
    #s = rospy.Service('getCentroid', Centroid, getCentroid)
    print "Service is active now!"
    rospy.spin()

    print "Complete!"
