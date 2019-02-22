#!/usr/bin/env python
# Master node for handling sensor updates and velocity commands

# Imports
import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import math
from fl_classifier import fl_classifier
from state import ExploreState, ChaseState, ClassifyState, IdleState, GoalState
from pid import PID
from state import Events, States
from random import randint


### USEFUL FUNCTIONS ###
def deg2rad(degrees):
    radians = (degrees / 180) * math.pi
    return radians


### CALLBACKS ###
# Callback from distance detector
def distanceCallback(dis_msg):
    global desired_distance
    global distance
    global distance_error
    global start
    global closeToSign

    desired_distance = 0.35

    distance = float(dis_msg.data)
    distance_error = distance - desired_distance
    start = True

    if abs(distance_error) < 0.02:
        closeToSign = True
    else:
        closeToSign = False


# Callback from object detector
def locationCallback(loc_msg):
    global desired_angle
    global angle
    global angle_error
    global start2
    global isSign
    global inFrontOfSign

    camera_fov= 66.2
    x_loc = loc_msg.x
    img_size = loc_msg.z
    pixel_error = img_size / 2 - x_loc
    angle_error = deg2rad(pixel_error * (camera_fov / img_size))

    start2 = True

    if x_loc > 800:
        isSign = False
    else:
        isSign = True

    if abs(angle_error) < math.pi/32:
        inFrontOfSign = True
    else:
        inFrontOfSign = False


# Callback from odometry
def odomCallback(odom_msg):
    global orientation
    a=1
    global angle
    # x_pos = odom_msg
    angle = odom_msg.pose.pose.orientation

    position = odom_msg.pose.pose.position
    # Orientation uses the quaternion aprametrization
    # To get the angular position along the z-axis, the following equation is required
    q = odom_msg.pose.pose.orientation
    orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    if (orientation < 0):
        orientation = math.pi + (math.pi + orientation)
    # print('orientation=', orientation)


### FUNCTIONS ###
# Check if sensors are activated
def checkIfActivated():
    if start is True and start2 is True:
        return True
    else:
        return False


def checkAtSign():
    if closeToSign and inFrontOfSign:
        return True
    else:
        return False


# Update current state
def stateUpdate(state):
    # start event as none
    event = Events.none

    # update states based on event
    # if state is IdleState:
    if state is States.Idle:
        print("state is IDLE")
        # event is system_activated
        activated = checkIfActivated()
        if activated:
            # event = Events.system_activated
            state = States.Explore
        # state = state.on_event(event)
        return state

    elif state is States.Explore:
        print("state is EXPLORE")

        # event is 'sign_found'
        if isSign:
            # event = Events.sign_found
            state = States.Chase
        # state = state.on_event(event)
        return state

    elif state is States.Chase:
        print("state is CHASE")

        # event is 'at_sign'
        atSign = checkAtSign()
        if atSign:
            state = States.Classify
        elif isSign is False and canDetect is True:
            state = States.Explore
        elif canDetect is False:
            state = States.Move
        # state = state.on_event(event)
        return state

    elif state is States.Classify:
        print("state is CLASSIFY")

        # event is 'action_complete'
        if actionComplete:
            # event = Events.action_completed
            state = States.Explore
        if atGoal:
            state = States.Goal
        # state = state.on_event(event)
        return state

    elif state is States.Move:
        print("state is Move")
        if canDetect:
            state = States.Chase
        return state

    elif state is States.Goal:
        print("state is GOAL")
        vels.linear.x = 0
        vels.angular.z = 0
        return state


def doExplore():
    # Spin
    lin_error = 0.0

    r = randint(0, 1)
    if 0 == r:
        ang_error =-math.pi/8
    elif 1 == r:
        ang_error = math.pi/8

    return ang_error, lin_error


def doChase():
    ang_error = angle_error

    if abs(ang_error) < math.pi / 16:
        lin_error = distance_error
    else:
        lin_error = 0.5
    return ang_error, lin_error


def classifySign():
    sign = classifier.classified_sign_int
    print('sign=', sign)
    global atGoal
    global sign_angle

    # if sign is ... do this
    if sign is 0:
        r = randint(0, 2)
        if 0 == r:
            sign_angle = math.pi / 2 + orientation
        elif 1 == r:
            sign_angle = math.pi / 2 + orientation
        else:
            sign_angle = math.pi + orientation
    elif sign is 1:  # LEFT TURN
        sign_angle = -math.pi / 2 + orientation
    elif sign is 2:  # RIGHT TURN
        sign_angle = math.pi / 2 + orientation
    # elif sign is 3:  # DEAD END
    #     sign_angle = math.pi + orientation
    elif sign is 4 or 3:  # STOP # DEAD END
        r=randint(0, 2)
        if 0==r:
            sign_angle = math.pi/2 + orientation
        elif 1==r:
            sign_angle = math.pi/2+ orientation
        else:
            sign_angle = math.pi+ orientation
    elif sign is 5:  # GOAL
        sign_angle = orientation
        atGoal = True
    else:
        sign_angle = orientation

    if sign_angle < 0:
        sign_angle = math.pi * 2 + sign_angle
    elif sign_angle > math.pi * 2:
        sign_angle = sign_angle - math.pi * 2


def doClassify():
    global actionComplete
    global isClassified
    ang_error =0
    lin_error = 0
    if isClassified == False:
        print('I am classifying')
        actionComplete=False
        classifySign()
        isClassified = True


    if actionComplete == False:
        print('I am moving')
        ang_error = (orientation - sign_angle)

        if (ang_error > math.pi):
            ang_error = -(ang_error - math.pi)
        if (ang_error < -math.pi):
            ang_error = -(ang_error + math.pi)


        if abs(ang_error) < math.pi / 16:
            actionComplete = True
            isClassified = False

    else:
        ang_error = 0
        print("error is 0")

    print('angle error in classify= ', ang_error)
    return ang_error, lin_error


def doMove():
    global canDetect
    global distance

    lin_error = distance - 0.3

    if lin_error < 0.3:
        canDetect = True
    else:
        canDetect = False
    return lin_error


def calculateErrors(state):
    global actionComplete
    # Calculate error depending on state
    if state is States.Idle:
        ang_error = 0.0
        lin_error = 0.0
    elif state is States.Explore:
        ang_error, lin_error = doExplore()
    elif state is States.Chase:
        ang_error, lin_error = doChase()
    elif state is States.Classify:
        ang_error, lin_error = doClassify()
    elif state is States.Move:
        lin_error = doMove()
        ang_error = 0.0
    elif state is States.Goal:
        ang_error = 0.0
        lin_error = 0.0
        vels.linear.x = 0
        vels.angular.z = 0
    else:
        ang_error = 0.0
        lin_error = 0.0
    return ang_error, lin_error


# Calculate velocities based on current state and error
def calculateVels(bot_state, vels, pid_lin, pid_ang):
    ang_error, lin_error = calculateErrors(bot_state)
    vels.angular.z = pid_ang.calculate(ang_error)
    vels.linear.x = pid_lin.calculate(lin_error)

    if bot_state is States.Goal:
        vels.angular.z = 0.0
        vels.linear.x = 0.0

    return vels


### MAIN FUNCTION ###
if __name__ == '__main__':

    # Setup ROS
    rospy.init_node('commander', anonymous=True)
    distance_sub = rospy.Subscriber("object_distance", Float32, distanceCallback)
    location_sub = rospy.Subscriber("image_location", Point, locationCallback)
    odom_sub = rospy.Subscriber("odom", Odometry, odomCallback)
    vels_pub = rospy.Publisher('cmd_vel',Twist, queue_size=5)
    update_rate = 5
    rate = rospy.Rate(update_rate)

    # start boolean for object detection
    start = False
    start2 = False
    isSign = False
    closeToSign = False
    inFrontOfSign = False
    atGoal = False
    actionComplete = False
    isClassified = False
    canDetect = False

    # Initial state for bot
    # bot_state = IdleState
    bot_state = States.Idle
    # desired distance and angle
    desired_distance = 0.35
    desired_angle = 0.0

    # Setup PID controller
    pid_linear = PID(1.0/update_rate,0.2, -0.1, 0.1, 0.05, 0.0)
    pid_angular = PID(1.0/update_rate, 0.8, -0.8, 1.0, 0.25, 0.01)

    # Setup initial velocities
    v = 0
    w = 0
    vels = Twist()
    vels.linear.x = v
    vels.angular.z = w

    classifier = fl_classifier()

    # ROS Loop
    while not rospy.is_shutdown():
        # Calculate new velocities
        bot_state = stateUpdate(bot_state)
        vels = calculateVels(bot_state, vels, pid_linear, pid_angular)

        # Publish new velocities
        vels_pub.publish(vels)

        # Sleep for subscriber updates
        rate.sleep()

