#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from kalman.msg import Kalman
from kalman.msg import RealData
import math

Q = 1
P = 10
R = 13

filterValue_x = 0
filterValue_y = 0

dt = 0.2
def callback(data):
    global predictValue_x
    global predictValue_y
    global filterValue_x
    global filterValue_y
                         
    predictValue_x = 1*filterValue_x + data.ax*dt
    predictValue_y = 1*filterValue_y + data.ay*dt
    P = P + Q;
    K = P / (P+R)
    filterValue_x = predictValue_x + (data.v*math.cos(data.theta)-predictValue_x)*K
    filterValue_y = predictValue_y + (data.v*math.sin(data.theta)-predictValue_y)*K
    P = (1 - K)*P 
    
    msg = RealData()
    msg.vx = filterValue_x
    msg.vy = filterValue_y
    pub.publish(msg)
    
def kalman():
  
    rospy.init_node('kalmanFilter', anonymous=True)
    sub = rospy.Subscriber("kalman_topic", Kalman, callback)
    pub = rospy.Publisher('kalman_pub', RealData, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    try:
        kalman()
    except rospy.ROSInterruptException:
        pass
     
