#!/usr/bin/env python

import sys
import rospy, time, tf
import math as math
from nav_msgs.msg import Odometry, OccupancyGrid
from final_project.srv import *
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Point
from tf.transformations import euler_from_quaternion
from threading import Thread


DEBUG_MODE = True
WHEEL_RADIUS = 0.035
DISTANCE_BETWEEN_WHEELS = 0.23
POS_TOLERANCE = 0.1
ANGLE_TOLERANCE = 0.05

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

#Requests the current position as a transform
def request_pos():
    global current_x
    global current_y
    global current_theta
    global receivedInitialPos

    rate = rospy.Rate(10.0)

    while not reachedGoal and not rospy.is_shutdown():
        try:
            (trans, rot) = tfListener.lookupTransform('map', 'base_footprint', rospy.Time(0))
            current_x = trans[0]
            current_y = trans[1]

            q = [rot[0], rot[1], rot[2], rot[3]]

            roll, pitch, yaw = euler_from_quaternion(q)

            current_theta = yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        if not receivedInitialPos:
            receivedInitialPos = True

        rate.sleep()

#Adds two angles
def addAngles(angle1, angle2):
    result = angle1 + angle2

    if result > math.pi:
        result %= math.pi
        result -= math.pi
    elif result < -math.pi:
        result %= -math.pi
        result += math.pi

    if DEBUG_MODE:
        print "Angle1: %f, Angle2: %f, Result: %f" % (angle1, angle2, result)

    return result

#Publishes twist
def publishTwist(x_vel, angular_vel):
    twist = Twist()
    twist.linear.x = x_vel;
    twist.linear.y = 0;
    twist.linear.z = 0
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = angular_vel

    pub.publish(twist)

# This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, t):
    x_vel = (WHEEL_RADIUS / 2) * (u1 + u2)
    angular_vel = (WHEEL_RADIUS / DISTANCE_BETWEEN_WHEELS) * (u1 - u2)

    startTime = time.time()

    publishTwist(x_vel, angular_vel)

    while time.time() - startTime < t:
        publishTwist(x_vel, angular_vel)

    publishTwist(0, 0)

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    startPos_x = current_x
    startPos_y = current_y

    while math.sqrt(math.pow(current_x - startPos_x, 2) + math.pow(current_y - startPos_y, 2)) < distance:
        if isNewTrajectoryReady:
            return

        publishTwist(speed, 0)

        rospy.sleep(rospy.Duration(0, 1))

    publishTwist(0, 0)

#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    destination_angle = addAngles(current_theta, angle)

    rotateToAngle(destination_angle)

#Rotates to the specified angle in the global coordinate frame
def rotateToAngle(destination_angle):
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
        if isNewTrajectoryReady:
            return

        publishTwist(0, angular_vel_dir*angular_vel)

        rospy.sleep(rospy.Duration(0, 1))

    publishTwist(0, 0)

#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    angular_vel = speed / radius

    if angle < 0:
        angular_vel = -angular_vel

    destination_angle = addAngles(current_theta, angle)

    while current_theta > destination_angle + ANGLE_TOLERANCE or current_theta < destination_angle - ANGLE_TOLERANCE:
        publishTwist(speed, angular_vel)

        rospy.sleep(rospy.Duration(0, 1))

    publishTwist(0, 0)

#Commands the robot to go to the position specified
def goToPosition(goalX, goalY):
    global current_x
    global current_y
    global current_theta

    xDiff = goalX - current_x
    yDiff = goalY - current_y

    # adding current_theta is done in rotate(angle)
    angle = math.atan2(yDiff, xDiff)
    rotateToAngle(angle)

    distance = math.sqrt(pow(xDiff, 2) + pow(yDiff, 2))
    print "Driving forward by distance: %f" % distance
    driveStraight(.15, distance)

#Callback function that processes the OccupancyGrid message.
def mapCallback(mapMessage):
    global map
    global receivedNewMap

    #Store the mapMessage as global in order for the requestTrajectory function to use it.
    map = mapMessage

    receivedNewMap = True

#Processes the received map.
def processReceivedMap():
    global receivedNewMap
    global isNewTrajectoryReady
    global previousTrajectory

    while not reachedGoal and not rospy.is_shutdown():
        if not receivedNewMap:
            continue
        else:
            #Reset flag
            receivedNewMap = False

        requestTrajectory()

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

#Requests the trajectory
def requestTrajectory():
    global trajectory

    initPos = PoseWithCovarianceStamped()
    initPos.pose.pose.position.x = current_x
    initPos.pose.pose.position.y = current_y

    goalPos = PoseStamped()
    goalPos.pose.position.x = goalPosX
    goalPos.pose.position.y = goalPosY

    rospy.wait_for_service('calculateTrajectory')

    try:
        calculateTraj = rospy.ServiceProxy('calculateTrajectory', Trajectory)
        trajectory = calculateTraj(initPos, goalPos, map)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
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

            goToPosition(localGoalX, localGoalY)

            if isNewTrajectoryReady:
                break

            if counter >= (len(oldTrajectoryPoses) - 1):
                reachedGoal = True

            counter += 1

if __name__ == "__main__":
    #Initialize the new node
    rospy.init_node('control')

    #Flags that indicate if the initial or goal positions were received
    global receivedInitialPos
    global receivedNewMap

    #Flag
    global isNewTrajectoryReady
    global reachedGoal

    #Flag that indicate whether the local goal was defined
    global wasLocalGoalDefined

    #Trajectory
    global trajectory
    global tfListener

    wasLocalGoalDefined = False
    reachedGoal = False
    isNewTrajectoryReady = False
    receivedInitialPos = False
    receivedNewMap = False

    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
    map_sub = rospy.Subscriber('/map', OccupancyGrid, mapCallback, queue_size=1)
    tfListener = tf.TransformListener()

    print "Waiting for the initial and goal positions..."

    Thread(target=request_pos).start()

    while not receivedInitialPos or not receivedNewMap:
        rospy.sleep(.1)
        pass

    #################################################################
    rospy.wait_for_service('getCentroid')

    try:
        getCentroid = rospy.ServiceProxy('getCentroid', Centroid)
        centroid = getCentroid(map)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

    # try:
    #     getTrajectory = rospy.ServiceProxy('getCentroid', T)
    #     trajectory = getCentroid(initPos, goalPos, map)
    # except rospy.ServiceException, e:
    #     print "Service call failed: %s" % e

    rospy.spin()

    exit()
    #################################################################

    #
    # print "Received initial and goal positions!"
    #
    # Thread(target=processReceivedMap).start()
    #
    # print "Starting trajectory execution..."
    #
    # executeTrajectory()

    print "Finished trajectory!"




