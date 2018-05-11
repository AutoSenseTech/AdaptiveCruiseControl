#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

desired_trajectory = 1
vel = 0

pub = rospy.Publisher('error', pid_input, queue_size=10)


##	Input: 	data: Lidar scan data
##			theta: The angle at which the distance is requried
##	OUTPUT: distance of scan at angle theta
def getRange(data, theta):
    # Find the index of the arary that corresponds to angle theta.
    # Return the lidar scan value at that index
    # Do some error checking for NaN and ubsurd values
    ## Your code goes here
    dist = data.ranges[int((theta - 90 + 135) / 0.25)]

    if dist > 10:
        dist = 10
    else:
        dist = dist
    return dist


def callback(data):
    theta = 50;
    a = getRange(data, theta)
    b = getRange(data, 0)
    swing = math.radians(theta)
    print a,b
    ## Your code goes here
    alpha = math.atan((a * math.cos(swing) - b)/(a * math.sin(swing)))
    AB = b * math.cos(alpha)
    AC = 0.1 # need to change
    CD = AB + AC * math.sin(alpha)
    error = CD - 1 # need to change
    print error
    ## END

    msg = pid_input()
    msg.pid_error = error
    msg.pid_vel = vel
    pub.publish(msg)


if __name__ == '__main__':
    print("Laser node started")
    rospy.init_node('dist_finder', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()
