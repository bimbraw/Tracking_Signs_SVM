#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int64,Float64
from sensor_msgs.msg import LaserScan
import sys
import math
# from gilleron_navigate_to_goal.msg import objectLocation



# class objectLocation(object):
#     angle_min = 0
#     angle_max = 1.5
#     distance = 100
#     def __init__(self,angle_min, angle_max, distance):
#         self.angle_min =angle_min
#         self.angle_max=angle_max
#         self.distance=distance

class obstacleDetection():
    def __init__(self):

        self.lidarInfo = rospy.Subscriber('/scan',LaserScan, self.obstacle_callback)


        self.objectBool = False

        self.obstacle_loc_pub = rospy.Publisher('/obstacle',Vector3,queue_size=1)

    def obstacle_callback(self, rosdata):
        length = len(rosdata.ranges)
        obstacle_loc = Vector3()
        min_dist = 100
        min_index = 0

        for i in range(0, length-1):
            dist = rosdata.ranges[i]
            if dist >0.1 and dist <1.0:
                self.objectBool = True
                if dist<min_dist:
                    min_dist=dist
                    min_index = i
        if self.objectBool:
            if self.deg2rad(min_index)>math.pi:
                obstacle_loc.x =-(2*math.pi - self.deg2rad(min_index))
            else:
                obstacle_loc.x = self.deg2rad(min_index)
            # print('angle min = ',obstacle_loc.x)
            obstacle_loc.z = min_dist
        else:
            obstacle_loc.x=1000 # angle min
            obstacle_loc.y=1000 # angle max
            obstacle_loc.z=1000 # distance
            self.objectBool = False

        self.obstacle_loc_pub.publish(obstacle_loc)

    def deg2rad(self,a):
        return a *math.pi/180

def main(args):
    ic = obstacleDetection()
    rospy.init_node('obstacleDetection', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down  obstacleDetection Range')

if __name__ == '__main__':
    main(sys.argv)
