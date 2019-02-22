#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np


def callback(msg):
    global distance
    global start

    distances = np.empty(0, float)
    length = len(msg.ranges)
    for i in xrange(0,4,1):
        dis = float(msg.ranges[i])
        distances = np.append(distances, dis)

    for i in (xrange(length-4, length)):
        dis = float(msg.ranges[i])
        distances = np.append(distances, dis)

    distance = np.average(distances)
    if distance > 3.0:
        distance = 3.0

    start = True

if __name__ == '__main__':
    rospy.init_node('object_distance', anonymous=True)
    laser_sub = rospy.Subscriber("scan", LaserScan, callback)
    distance_pub = rospy.Publisher('object_distance',Float32, queue_size=1)
    rate = rospy.Rate(5)

    start = False

    while not rospy.is_shutdown():
        if (start):
            distance_pub.publish(distance)
        rate.sleep()
