#!/usr/bin/env python

import sys
import rospy, time, tf
import math as math
from nav_msgs.msg import Odometry, OccupancyGrid
from final_project.srv import *
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
from threading import Thread

WHEEL_RADIUS = 0.035
DISTANCE_BETWEEN_WHEELS = 0.23
POS_TOLERANCE = 0.1
ANGLE_TOLERANCE = 0.05
POS_REQUEST_RATE = 10.0
PROCESS_COSTMAP = False

#Impelements PID controller
class PID:
    def __init__(self, P=1, I=0.0001, D=0.001, Derivator=0, Integrator=0, outMax=3, outMin=-3):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.outMax = outMax
        self.outMin = outMin

        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        self.I_value = self.Integrator * self.Ki

        if self.I_value > self.outMax:
            self.I_value = self.outMax
            self.Integrator -= self.error
        elif self.I_value < self.outMin:
            self.I_value = self.outMin
            self.Integrator -= self.error

        PID = self.P_value + self.I_value + self.D_value

        if PID > self.outMax:
            PID = self.outMax
        elif PID < self.outMin:
            PID = self.outMin

        return PID

    def setPoint(self, set_point):
        self.set_point = set_point
        self.Integrator = 0
        self.Derivator = 0

#A class that is responsible for the robot control and publishing messages to the robot
class RobotControl:
    def __init__(self, publisher):
        self.publisher = publisher

    #Adds two angles
    def addAngles(self, angle1, angle2):
        result = angle1 + angle2

        if result > math.pi:
            result %= math.pi
            result -= math.pi
        elif result < -math.pi:
            result %= -math.pi
            result += math.pi

        return result

    #Publishes twist
    def publishTwist(self, x_vel, angular_vel):
        twist = Twist()
        twist.linear.x = x_vel;
        twist.linear.y = 0;
        twist.linear.z = 0
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = angular_vel

        self.publisher.publish(twist)

    # This function accepts two wheel velocities and a time interval.
    def spinWheels(self, u1, u2, t):
        x_vel = (WHEEL_RADIUS / 2) * (u1 + u2)
        angular_vel = (WHEEL_RADIUS / DISTANCE_BETWEEN_WHEELS) * (u1 - u2)

        startTime = time.time()

        self.publishTwist(x_vel, angular_vel)

        while time.time() - startTime < t:
            self.publishTwist(x_vel, angular_vel)

        self.publishTwist(0, 0)

    #This function accepts a speed and a distance for the robot to move in a straight line
    def driveStraight(self, speed, distance):
        startPos_x = current_x
        startPos_y = current_y

        while math.sqrt(math.pow(current_x - startPos_x, 2) + math.pow(current_y - startPos_y, 2)) < distance:
            self.publishTwist(speed, 0)

            rospy.sleep(rospy.Duration(0, 1))

        self.publishTwist(0, 0)

    #Accepts an angle and makes the robot rotate around it.
    def rotate(self, angle):
        destination_angle = self.addAngles(current_theta, angle)

        self.rotateToAngle(destination_angle)

    #Rotates to the specified angle in the global coordinate frame
    def rotateToAngle(self, destination_angle):
        angular_vel = 0.5

        if destination_angle > current_theta:
            if destination_angle - current_theta < math.pi:
                angular_vel_dir = 1
            else:
                angular_vel_dir = -1
        else:
            if current_theta - destination_angle < math.pi:
                angular_vel_dir = -1
            else:
                angular_vel_dir = 1

        while current_theta > destination_angle + ANGLE_TOLERANCE or current_theta < destination_angle - ANGLE_TOLERANCE:
            self.publishTwist(0, angular_vel_dir*angular_vel)

            rospy.sleep(rospy.Duration(0, 1))

        self.publishTwist(0, 0)

    #Rotate 360 degrees
    def rotateAround(self):
        self.rotate(math.pi/2)
        self.rotate(math.pi/2)
        self.rotate(math.pi/2)
        self.rotate(math.pi/2)

    #This function works the same as rotate how ever it does not publish linear velocities.
    def driveArc(self, radius, speed, angle):
        angular_vel = speed / radius

        if angle < 0:
            angular_vel = -angular_vel

        destination_angle = self.addAngles(current_theta, angle)

        while current_theta > destination_angle + ANGLE_TOLERANCE or current_theta < destination_angle - ANGLE_TOLERANCE:
            self.publishTwist(speed, angular_vel)

            rospy.sleep(rospy.Duration(0, 1))

        self.publishTwist(0, 0)

    #Commands the robot to go to the position specified
    def goToPosition(self, goalX, goalY):
        global current_x
        global current_y
        global current_theta

        xDiff = goalX - current_x
        yDiff = goalY - current_y

        # adding current_theta is done in rotate(angle)
        angle = math.atan2(yDiff, xDiff)
        self.rotateToAngle(angle)

        distance = math.sqrt(pow(xDiff, 2) + pow(yDiff, 2))
        print "Driving forward by distance: %f" % distance
        self.driveStraight(.15, distance)

#Requests the current position as a transform at rate of 10Hz
def request_pos_at_rate(frequency):
    global current_x
    global current_y
    global current_theta
    global receivedInitPos

    tfListener = tf.TransformListener()

    rate = rospy.Rate(frequency)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = tfListener.lookupTransform('map', 'base_footprint', rospy.Time(0))
            current_x = trans[0]
            current_y = trans[1]

            q = [rot[0], rot[1], rot[2], rot[3]]

            roll, pitch, yaw = euler_from_quaternion(q)

            current_theta = yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        if not receivedInitPos:
            receivedInitPos = True

        rate.sleep()

#Callback function that processes the OccupancyGrid message.
def mapCallback(mapMessage):
    global receivedNewMap
    global map

    #Store the mapMessage as global in order for the requestTrajectory function to use it.
    map = mapMessage

    receivedNewMap = True

#Processes the received map.
def requestTrajectory(goalPos):
    global receivedNewMap
    global isNewTrajectoryReady
    global previousTrajectory
    global trajectory

    while not reachedGoal and not rospy.is_shutdown():
        if not receivedNewMap:
            continue
        else:
            #Reset flag
            receivedNewMap = False

        #Request new trajectory
        initPos = Point()
        initPos.x = current_x
        initPos.y = current_y

        #create a stub for costmap
        if not PROCESS_COSTMAP:
            costMap = OccupancyGrid()

        trajectory = getTrajectory(initPos, goalPos, map, Bool(data=PROCESS_COSTMAP), costMap)

        #Check if the previous trajectory was defined
        try:
            previousTrajectory
        except NameError:
            previousTrajectory = trajectory
            isNewTrajectoryReady = True
        else:
            #Check if the previous trajectory is the same as the received trajectory
            if previousTrajectory.path.poses != trajectory.path.poses:
                isNewTrajectoryReady = True

#This function sequentially calls methods to perform a trajectory.
def executeTrajectory(control):
    global reachedGoal
    global isNewTrajectoryReady
    global localGoalX
    global localGoalY
    global wasLocalGoalDefined

    #Wait for the initial trajectory
    while not isNewTrajectoryReady:
        pass

    while not reachedGoal and not rospy.is_shutdown():
        counter = 0
        oldTrajectoryPoses = trajectory.path.poses
        isNewTrajectoryReady = False

        for point in oldTrajectoryPoses:
            localGoalX = point.pose.position.x
            localGoalY = point.pose.position.y

            wasLocalGoalDefined = True

            control.goToPosition(localGoalX, localGoalY)

            if isNewTrajectoryReady:
                break

            if counter >= (len(oldTrajectoryPoses) - 1):
                reachedGoal = True

            counter += 1

#Executes the main task of exploring the world around
def exploreEnvironment():
    teleop_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
    control = RobotControl(teleop_pub)

    while not rospy.is_shutdown():
        #1) Rotate 360 degrees to initially explore the world around
        print "=====> Started new iteration <====="
        print "1) Rotating 360."
        control.rotateAround()

        #2) Request new centroid
        print "2) Requesting new centroid."
        centroidResponse = getCentroid(map)
        if not centroidResponse.foundCentroid.data:
            #We are done, exit.
            break

        #3) Go to a new goal
        print "3) Navigating to the centroid."
        navigateToGoal(control, centroidResponse.centroid)
        print "======>   Ended iteration   <====="

def navigateToGoal(control, goal):
    global reachedGoal

    reachedGoal = False

    Thread(target=requestTrajectory, name="requestTrajectory() Thread", args=[goal]).start()
    executeTrajectory(control)

if __name__ == "__main__":
    #Initialize the new node
    rospy.init_node('control')

    #Wait for trajectory service to start up
    print "Waiting for getTrajectory() service to start up...",
    rospy.wait_for_service('getTrajectory')
    global getTrajectory
    getTrajectory = rospy.ServiceProxy('getTrajectory', Trajectory)
    print "DONE"
    # trajectory = getCentroid(initPos, goalPos, map, processCostMap, costMap)
    # initPos.pose.pose.position
    print "Waiting for getCentroid() service to start up...",
    #Wait for centroid service to start up
    rospy.wait_for_service('getCentroid')
    global getCentroid
    getCentroid = rospy.ServiceProxy('getCentroid', Centroid)
    print "DONE"
    # centroid = getCentroid(map)

    #Flags that indicate if the initial or goal positions were received
    global receivedInitPos
    global receivedNewMap

    #Flag
    global isNewTrajectoryReady
    global reachedGoal

    #Flag that indicate whether the local goal was defined
    global wasLocalGoalDefined

    #Trajectory
    global trajectory
    global map

    wasLocalGoalDefined = False
    reachedGoal = False
    isNewTrajectoryReady = False
    receivedInitPos = False
    receivedNewMap = False

    #Subscribe to map updates
    map_sub = rospy.Subscriber('/map', OccupancyGrid, mapCallback, queue_size=1)
    #Start requesting position in background
    Thread(target=request_pos_at_rate, name="Request_pos_at_rate Thread", args=[POS_REQUEST_RATE]).start()

    print "Waiting for the initial position from tf...",

    while not receivedInitPos or not receivedNewMap:
        rospy.sleep(.1)
        pass

    print "DONE"

    print "Started exploration."

    # control = RobotControl(rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5))
    # control.rotateAround()

    exploreEnvironment()

    print "Done with exploration. Exiting...",
    print "DONE :)"
    exit()


