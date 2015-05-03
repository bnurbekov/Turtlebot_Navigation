#!/usr/bin/env python

import rospy, math, heapq, Queue, threading
# Add additional imports for each of the message types used
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData, GridCells, Path
from std_msgs.msg import Bool
from final_project.srv import *
import gc

SCALE_FACTOR = 4

#Class that implements priority queue.
class PriorityQueue:
    #Initializes priority queue
    def __init__(self):
        self.elements = []

    #Checks if the queue is empty
    def empty(self):
        return len(self.elements) == 0

    #Puts the item with the given priority into the priority queue
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    #Pops the first item off the priority queue
    def get(self):
        return heapq.heappop(self.elements)[1]

#Class that implements grid.
class Grid:
    #Initializes the grid
    def __init__(self, data, height, width, resolution, origin):
        self.height = height
        self.width = width
        self.resolution = resolution
        self.origin = origin

        (x, y) = origin
        self.cellOrigin = (x + resolution/2, y + resolution/2)

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

    #Prints the grid (primarily used for debugging).
    def printToConsole(self):
        for i in range(0, self.height):
            for j in range(0, self.width):
                cell = (j, self.height - 1 - i)
                print self.getCellValue(cell),
            print ""

    #Abstract method that scales the map by the scaleFactor.
    def scale(self, scaleFactor):
        raise NotImplementedError("scale() method is not implemented!")

    #Populates the set with cells that have the given value
    @staticmethod
    def populateSetWithCells(grid, set, value):
        for i in range(0, grid.height):
            for j in range(0, grid.width):
                cell = (j, i)
                cellType = grid.getCellValue(cell)
                if cellType == value:
                    set.add(cell)

#Class that implements occupancy grid
class OccupancyGrid(Grid):
    #Initializes occupancy grid
    def __init__(self, data, height, width, resolution, origin):
        Grid.__init__(self, data, height, width, resolution, origin)

        #Creates the sets for obstacles and empty cells (for faster obstacle expansion and frontier searching)
        self.obstacles = set()
        self.empty = set()

        self.costGrid = None

    #Gets neighbors of the specific cell
    def getNeighbors(self, cell, includeObstacles=False):
        neighborList = []

        (x, y) = cell

        for i in range(0, 3):
            for j in range(0, 3):
                neighborCell = (x - 1 + j, y - 1 + i)

                if includeObstacles:
                    if self.isWithinGrid(neighborCell) and cell != neighborCell:
                        neighborList.append(neighborCell)
                else:
                    if self.isWithinGrid(neighborCell) and cell != neighborCell and self.getCellValue(neighborCell) != CellType.Obstacle:
                        neighborList.append(neighborCell)

        return neighborList

    #Gets neighbors of the specific cell
    def getNeighborValues(self, cell):
        neighbors = self.getNeighbors(cell, True)
        neighborValues = []

        for neighbor in neighbors:
            neighborValue = self.getCellValue(neighbor)
            neighborValues.append(neighborValue)

        return neighborValues

    #Gets neighbors and their respective values as list of tuples
    def getNeighborValuePairs(self, cell):
        neighbors = self.getNeighbors(cell, True)
        neighborValuePairs = []

        for neighbor in neighbors:
            neighborValue = self.getCellValue(neighbor)
            neighborValuePair = (neighbor, neighborValue)

            neighborValuePairs.append(neighborValuePair)

        return neighborValuePairs

    #Scales map to a new resolution
    def scale(self, scaleFactor, cacheEmptyCells=True, cacheObstacleCells=True):
        self.obstacles.clear()
        self.empty.clear()

        if type(scaleFactor) != int:
            raise Exception("The scale factor should be an integer!")

        if scaleFactor < 1:
            raise Exception("New resolution should be larger than the old resolution!")

        ng_data = [] #ng stands for NewGrid
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
                    if cacheObstacleCells:
                        self.obstacles.add((ng_column, ng_row))
                    if cacheEmptyCells and ng_oldCellValue == CellType.Empty:
                        self.empty.remove((ng_column, ng_row))

                elif (currentCellValue == CellType.Unexplored):
                    if ng_oldCellValue != CellType.Obstacle:
                        ng_data[ng_row][ng_column] = CellType.Unexplored
                        if cacheEmptyCells and ng_oldCellValue == CellType.Empty:
                            self.empty.remove((ng_column, ng_row))

                else: #empty cell
                    if ng_oldCellValue != CellType.Obstacle and ng_oldCellValue != CellType.Unexplored:
                        ng_data[ng_row][ng_column] = CellType.Empty
                        if cacheEmptyCells:
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

        if self.empty: #If the empty cell cache is not empty, then remove empty cells that turned into obstacles after expansion
            self.empty = self.empty - self.obstacles

    #Adds cost grid to allow local cost map processing
    def addCostGrid(self, costGrid):
        self.costGrid = costGrid

        if (self.resolution != costGrid.resolution):
            raise Exception("Current implemenation does not support the addition of the cost grid that has different "
                            "resolution than the occupancy grid.")

        #Warning: The offset calculation
        self.costGridCellOffset = (int((costGrid.cellOrigin[0] - self.cellOrigin[0]) / self.resolution),
                                   int((costGrid.cellOrigin[1] - self.cellOrigin[1]) / self.resolution))

    #Gets heuristic relevant to the current grid
    def getHeuristic(self, currentCell, destinationCell):
        (currentX, currentY) = currentCell
        (destinationX, destinationY) = destinationCell

        return math.sqrt((currentX - destinationX) ** 2 +
                         (currentY - destinationY) ** 2)

    #Gets path cost relevant to the current grid
    def getPathCost(self, currentCell, destinationCell):
        cost = 0

        (currentX, currentY) = currentCell
        (destinationX, destinationY) = destinationCell

        xDiff = abs(currentX - destinationX)
        yDiff = abs(currentY - destinationY)

        if (xDiff > 1 or xDiff < 0) or (yDiff > 1 or yDiff < 0):
            raise Exception("getPathCost: The function estimates the cost only for adjacent cells!")

        if xDiff + yDiff == 2:
            cost += 1.4
        elif xDiff + yDiff == 1:
            cost += 1

        if self.costGrid != None:
            #find the corresponding cell in costGrid
            costGridCell = (currentCell[0] - self.costGridCellOffset[0], currentCell[1] - self.costGridCellOffset[1])

            #add the additional cost if the cell is in costmap
            if self.costGrid.isWithinGrid(costGridCell):
                cost += self.costGrid.getCellValue(costGridCell)

        return cost

#Class that implements cost grid
class CostGrid(Grid):
    #Initializes occupancy grid
    def __init__(self, data, height, width, resolution, origin):
        Grid.__init__(self, data, height, width, resolution, origin)

    #Scales the occupancy grid by the factor provided
    def scale(self, scaleFactor):
        if type(scaleFactor) != int:
            raise Exception("The scale factor should be an integer!")

        if scaleFactor < 1:
            raise Exception("New resolution should be larger than the old resolution!")

        ng_data = [] #ng stands for NewGrid
        ng_resolution = self.resolution * scaleFactor

        #Round up the new width and height
        ng_width = -(-self.width // scaleFactor)
        ng_height = -(-self.height // scaleFactor)

        #Since the cells are squares, then the maximum number of cells in the old grid per cell in the new grid is going
        #to be the square of the scaleFactor
        max_cells_per_ng_cell = scaleFactor ** 2

        ng_row = -1

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
                    ng_data[ng_row].append([0, 0]) # -2 indicates that the new cell has no value assigned to it yet

                cellValue = self.getCellValue((j, i))
                ng_cellValue = ng_data[ng_row][ng_column]

                ng_cellValue[0] += cellValue
                ng_cellValue[1] += 1

                [ng_sum, ng_count] = ng_cellValue

                #If the max count of cells per new grid cell was reached, then replace sum and count pair with the actual average value
                if (ng_count >= max_cells_per_ng_cell):
                    ng_data[ng_row][ng_column] = ng_sum/ng_count

        #Iterate through the last row and column to make sure all the sum and count pairs were converted into the actual average
        for i in range(0, ng_height):
            if (i == ng_height - 1):
                for j in range(0, ng_width):
                    ng_cellValue = ng_data[i][j]
                    if type(ng_cellValue) == list:
                        [ng_sum, ng_count] = ng_cellValue
                        ng_data[i][j] = ng_sum/ng_count
            else:
                ng_cellValue = ng_data[i][ng_width-1]
                if type(ng_cellValue) == list:
                    [ng_sum, ng_count] = ng_cellValue
                    ng_data[i][j] = ng_sum/ng_count

        self.data = ng_data
        self.height = ng_height
        self.width = ng_width
        self.resolution = ng_resolution

        (x, y) = self.origin
        self.cellOrigin = (x + ng_resolution/2, y + ng_resolution/2)

#Class that implements path finding functionality
class PathFinder:
    #Initializes PathFinder class
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
                new_cost = self.cost_so_far[current] + grid.getPathCost(current, neighbor)
                if neighbor not in self.cost_so_far or new_cost < self.cost_so_far[neighbor]:
                    self.cost_so_far[neighbor] = new_cost
                    priority = new_cost + grid.getHeuristic(neighbor, self.goal)
                    self.frontier.put(neighbor, priority)
                    self.parent[neighbor] = current

        return False

    #Finds path from goal to destination using internal parent list
    def findPath(self):
        self.path = PathFinder.getPath(self.start, self.goal, self.parent)

    #Extracts waypoints from the path
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

    #Static method that returns a path from goal to destination based on the parent list provided
    @staticmethod
    def getPath(start, goal, parent):
        path = []
        current = goal

        while current != start:
            path.append(current)
            current = parent[current]

        path.append(current)
        path.reverse()

        return path

    #Finds a path to the cell with one of the values provided by goalCellValues. Additionally, tries to select the cell
    #closest to the cell specified by goal argument during the tie-breaking process.
    @staticmethod
    def findPathToCellWithValueClosestTo(grid, start, goalCellValues, goal):
        visited = set()
        queue = PriorityQueue()
        parent = {}
        queue.put(start, 0)
        visited.add(start)

        while not queue.empty():
            tuple = heapq.heappop(queue.elements)

            path_cost = tuple[0]
            current = tuple[1]

            # print "Popped: ", current, "Path cost -", path_cost, "Cell value -", grid.getCellValue(current)

            if grid.getCellValue(current) in goalCellValues:
                bestCellSoFar = current
                bestCellHeuristics = grid.getHeuristic(current, goal)

                #pop next cells with the same path cost
                next_path_cost = path_cost

                #find the cell with the same path cost, but better heuristic
                while path_cost == next_path_cost and not queue.empty():
                    tuple = heapq.heappop(queue.elements)

                    next_path_cost = tuple[0]
                    next = tuple[1]

                    tempHeuristics = grid.getHeuristic(next, goal)

                    if tempHeuristics < bestCellHeuristics and next in goalCellValues:
                        bestCellHeuristics = tempHeuristics
                        bestCellSoFar = next

                #Return path out of obstacle
                return PathFinder.getPath(start, bestCellSoFar, parent)
            else:
                neighbors = grid.getNeighbors(current, includeObstacles=True)

                # print "Adding neighbors!"
                for neighbor in neighbors:
                    # print "Neighbor: ", neighbor, "Cell value -", grid.getCellValue(neighbor)
                    if neighbor not in visited:
                        # print "Neighbor was added!"
                        queue.put(neighbor, path_cost + 1)
                        parent[neighbor] = current
                        visited.add(neighbor)

    #Finds the first cell not surrounded with the cell types specified by cellTypes argument. Performs the search only through
    #the cells with goalCellType provided.
    @staticmethod
    def findTheFirstCellNotSurroundedWith(grid, start, goalCellType, cellTypes):
        visited = set()
        queue = PriorityQueue()
        queue.put(start, 0)
        visited.add(start)

        while not queue.empty():
            tuple = heapq.heappop(queue.elements)

            path_cost = tuple[0]
            current = tuple[1]

            neighborValuePairs = grid.getNeighborValuePairs(current)
            neighborValues = [x[1] for x in neighborValuePairs]

            notSurrounded = True

            for neighborValue in neighborValues:
                if neighborValue in cellTypes:
                    notSurrounded = False

            if notSurrounded:
                return current
            else:
                neighborValuePairs = grid.getNeighborValuePairs(current)

                for neighborValuePair in neighborValuePairs:
                    (neighbor, neighborValue) = neighborValuePair

                    if neighbor not in visited and neighborValue == goalCellType:
                        queue.put(neighbor, path_cost + 1)
                        visited.add(neighbor)

        raise Exception("Was not able to find the cell not surrounded with cell types passed!")

#A class that has a function of cell type enumeration.
class CellType:
    Unexplored = -1
    Empty = 0
    Obstacle = 100

#Processes the received occupancy grid message.
def processOccupancyGrid(gridMessage, scaleFactor, cacheEmptyCells):
    grid = OccupancyGrid(gridMessage.data, gridMessage.info.height, gridMessage.info.width, gridMessage.info.resolution,
                (gridMessage.info.origin.position.x, gridMessage.info.origin.position.y))
    grid.scale(scaleFactor, cacheEmptyCells=cacheEmptyCells)
    grid.expandObstacles()

    return grid

#Processes the received cost grid messsage.
def processCostGrid(gridMessage, scaleFactor, result_queue):
    costGrid = CostGrid(gridMessage.data, gridMessage.info.height, gridMessage.info.width, gridMessage.info.resolution,
                (gridMessage.info.origin.position.x, gridMessage.info.origin.position.y))
    costGrid.scale(scaleFactor)
    result_queue.put(costGrid)
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

#Publishes grid cells contained in gridCellList to the topic specified by publisher argument.
def publishGridCells(publisher, gridCellList, resolution, cellOrigin):
    gridCellsMessage = createGridCellsMessage(resolution)

    for cell in gridCellList:
        gridCellsMessage.cells.append(convertCellToPoint(cell, cellOrigin, resolution))

    publisher.publish(gridCellsMessage)

#Creates a GridCells message and auto-initializes some fields.
def createGridCellsMessage(gridResolution):
    gridCells = GridCells()
    gridCells.header.seq = 0
    gridCells.header.stamp = rospy.Time.now()
    gridCells.header.frame_id = "map"
    gridCells.cell_width = gridResolution
    gridCells.cell_height = gridResolution

    gridCells.cells = []

    return gridCells

#Service function that calculates trajectory for the robot to follow in order to get to the goal cell.
def getTrajectory(req):
    global grid

    print "Received trajectory request."

    #Start the thread for the centroid finding logic
    if req.processCostMap.data:
        #Make sure that the occupancy grid and the cost map have the same resolution
        costGrid_scaleFactor = int(SCALE_FACTOR * (req.costMap.info.resolution / req.map.info.resolution))

        result_queue = Queue.Queue()
        thread1 = threading.Thread(
                target=processCostGrid,
                name="ProcessCostGrid() Thread",
                args=[req.costMap, costGrid_scaleFactor, result_queue],
                )
        thread1.start()

    print "Processing occupancy grid...",

    grid = processOccupancyGrid(req.map, SCALE_FACTOR, False) #We do not need to cache empty cells for trajectory calculation

    print "DONE"

    if req.processCostMap.data:
        thread1.join()
        costGrid = result_queue.get()
        grid.addCostGrid(costGrid)

    print "Getting waypoints...",

    #Meanwhile fine the path using A star in the main thread
    waypoints = getWaypoints(req.initPos, req.goalPos, grid)

    print "DONE"

    print "Done with the trajectory request processing!"

    gc.collect()

    return TrajectoryResponse(waypoints)

#Returns a list of waypoints after running A* algorithm to find a path from initPos to goalPos.
def getWaypoints(initPos, goalPos, grid):
    startCell = convertPointToCell(initPos, grid.origin, grid.resolution)
    goalCell = convertPointToCell(goalPos, grid.origin, grid.resolution)

    #Logic that gets the robot out of an obstacle (failure recovery)
    if grid.getCellValue(startCell) == CellType.Obstacle:
        print "The robot is inside the obstacle! Trying to find the shortest way out..."
        cellTypes = set()
        cellTypes.add(CellType.Empty)
        cellTypes.add(CellType.Unexplored)
        pathOutOfObstacle = PathFinder.findPathToCellWithValueClosestTo(grid, startCell, cellTypes, goalCell)

        startCell = pathOutOfObstacle.pop(len(pathOutOfObstacle) - 1)

    pathFinder = PathFinder(startCell, goalCell)

    if (startCell == goalCell):
        pathFinder.waypoints.append(goalCell)
    else:
        aStarDone = False

        while not aStarDone:
            aStarDone = pathFinder.runAStarIteration(grid)

        publishGridCells(expanded_cell_pub, pathFinder.expanded, grid.resolution, grid.cellOrigin)

        #Convert frontier queue to the frontier set
        frontierCells = set()

        for tuple in pathFinder.frontier.elements:
            cell = tuple[1]
            if cell not in pathFinder.expanded and cell not in frontierCells:
                frontierCells.add(cell)

        publishGridCells(frontier_cell_pub, frontierCells, grid.resolution, grid.cellOrigin)

        pathFinder.findPath()

        #if robot was at an obstacle cell before, then prepend the path out of the obstacle
        try:
            pathOutOfObstacle
        except NameError:
            pass
        else:
            pathOutOfObstacle.extend(pathFinder.path)
            pathFinder.path = pathOutOfObstacle

        publishGridCells(path_cell_pub, pathFinder.path, grid.resolution, grid.cellOrigin)

        pathFinder.calculateWaypoints()

    #convert the waypoints to the trajectory offsets:
    waypoints = Path()
    waypoints.poses = []

    for cell in pathFinder.waypoints:
        poseObj = PoseStamped()
        poseObj.pose.position = convertCellToPoint(cell, grid.cellOrigin, grid.resolution)

        waypoints.poses.append(poseObj)

    return waypoints

#Gets the centroid of the largest frontier
def getCentroid(req):
    global grid

    print "Received centroid request."

    grid = processOccupancyGrid(req.map, SCALE_FACTOR, True)

    foundCentroid = False

    visited = set()
    clusters = []

    print "Started to look for clusters..."

    for cell in grid.empty:
        cluster = []

        # rospy.logdebug("---------> New cluster!")

        expandCluster(cell, cluster, visited)

        if len(cluster) != 0:
            clusters.append(cluster)
            # rospy.logdebug("---------> Cluster was added!")
        # else:
        #     rospy.logdebug("---------> Cluster was not added!")

    print "DONE"

    print "Calculating centroid..."

    centroidPos = Point()

    if len(clusters) != 0:
        #Sort clusters by the number of elements in them in order to check the largest clusters first
        clusters.sort(key = lambda tup: len(tup), reverse=True)

        for currentCluster in clusters:
            centroid = calculateCentroid(currentCluster)
            centroidValue = grid.getCellValue(centroid)

            try:
                #if the centroid is inside the unexplored area or obstacle, then find the closest empty cell
                if centroidValue == CellType.Obstacle or centroidValue == CellType.Unexplored:
                    path = PathFinder.findPathToCellWithValueClosestTo(grid, centroid, [CellType.Empty], (req.currentPos.x, req.currentPos.y))
                    centroid = path.pop(len(path) - 1)

                #if the centroid is on the border with the unexplored cell, then find the first empty cell that is not
                centroid = PathFinder.findTheFirstCellNotSurroundedWith(grid, centroid, CellType.Empty, [CellType.Unexplored])

                #Check if get trajectory will throw an error
                centroidPos = convertCellToPoint(centroid, grid.cellOrigin, grid.resolution)

                getWaypoints(req.currentPos, centroidPos, grid)
            except Exception, e:
                print "Exception thrown during the centroid processing: ", str(e)
                print "Proceeding to the next cluster..."
                continue
            else:
                foundCentroid = True
                break

        ########################################Printing##################################################
        clusterCells = []
        for cluster in clusters:
            clusterCells += cluster


        # print "Number of clusters: %d" % (len(clusters))

        # Animation that shows the clusters and their respective centroids. Use for debugging!
        # for cluster in clusters:
        #     centr = calculateCentroid(cluster)
        #     publishGridCells(cluster_cell_pub, cluster, grid.resolution, grid.cellOrigin)
        #     publishGridCells(centroid_cell_pub, [centr], grid.resolution, grid.cellOrigin)
        #     rospy.sleep(5)

        publishGridCells(cluster_cell_pub, clusterCells, grid.resolution, grid.cellOrigin)
        publishGridCells(centroid_cell_pub, [centroid], grid.resolution, grid.cellOrigin)
        publishGridCells(empty_cell_pub, grid.empty, grid.resolution, grid.cellOrigin)
        publishGridCells(obstacle_cell_pub, grid.obstacles, grid.resolution, grid.cellOrigin)

    print "DONE"

    print "Done with the centroid request processing!"

    gc.collect()

    return CentroidResponse(Bool(data=foundCentroid), centroidPos)

#Adds the cell to the cluster if the cell is on the border with the unexplored cells and is not visited
def expandCluster(cell, cluster, visited):
    if cell in visited:
        return

    visited.add(cell)
    # "Visited:"
    # print cell
    possibleClusterCandidates = []

    neighbors = grid.getNeighbors(cell)
    # print "Neighbors:"
    # print neighbors

    doesCellBelongToCluster = False

    (x, y) = cell
    for neighbor in neighbors:
        neighborValue = grid.getCellValue(neighbor)
        (neighborX, neighborY) = neighbor

        # print neighbor,
        # print "=>",
        # print neighborValue

        if neighborValue == CellType.Unexplored:
            if (neighborX - x == 0 or neighborY - y == 0):
                # print "Unexplored cell:"
                # print neighbor
                doesCellBelongToCluster = True
        elif neighborValue == CellType.Empty:
            possibleClusterCandidates.append(neighbor)

    # print "Does cell belong to cluster:"
    # print doesCellBelongToCluster

    if doesCellBelongToCluster:
        #1) Add the cell to the cluster
        cluster.append(cell)
        #2) Check if the neighbor empty cells belong to this cluster as well
        for clusterCandidate in possibleClusterCandidates:
            expandCluster(clusterCandidate, cluster, visited)

#Calculates a centroid of a given cluster.
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

#Main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('mapping')

    global expanded_cell_pub
    global frontier_cell_pub
    global path_cell_pub
    global cluster_cell_pub
    global centroid_cell_pub
    global empty_cell_pub
    global obstacle_cell_pub
    global unknown_cell_pub

    expanded_cell_pub = rospy.Publisher('/expandedGridCells', GridCells, queue_size=5) # Publisher for commanding robot motion
    frontier_cell_pub = rospy.Publisher('/frontierGridCells', GridCells, queue_size=5) # Publisher for commanding robot motion
    path_cell_pub = rospy.Publisher('/pathGridCells', GridCells, queue_size=5)
    cluster_cell_pub = rospy.Publisher('/clusterGridCells', GridCells, queue_size=5)
    centroid_cell_pub = rospy.Publisher('/centroidGridCell', GridCells, queue_size=5)
    empty_cell_pub = rospy.Publisher('/emptyGridCell', GridCells, queue_size=5)
    obstacle_cell_pub = rospy.Publisher('/obstacleGridCell', GridCells, queue_size=5)
    unknown_cell_pub = rospy.Publisher('/unknownGridCell', GridCells, queue_size=5)

    print "Starting..."

    s = rospy.Service('getTrajectory', Trajectory, getTrajectory)
    print "getTrajectory() service is active now!"
    s = rospy.Service('getCentroid', Centroid, getCentroid)
    print "getCentroid() service is active now!"
    rospy.spin()

    print "Complete!"
