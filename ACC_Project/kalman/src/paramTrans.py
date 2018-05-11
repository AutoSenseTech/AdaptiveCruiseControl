#!/usr/bin/env python
import rospy
from kalman.msg import Kalman
from imu.msg import Accelerator
from race2.msg import drive_param
from rpm.msg import datarpm

ax = 0
ay = 0
theta = 0
v = 0

def callback1(data):
    global ax
    global ay
    
    ax = data.x
    ay = data.y


def callback2(data):
    global theta
    theta = data.angle / 3



def callback3(data):
    global v
    v = data.R / 4 * 0.316




def paramTrans():
    rospy.init_node('paramTrans', anonymous=True)
    sub1 = rospy.Subscriber('accelerator', Accelerator, callback1)
    sub2 = rospy.Subscriber('drive_parameters', drive_param, callback2)
    sub3 = rospy.Subscriber('RPM_TOPIC', datarpm, callback3)
    pub= rospy.Publisher('kalman_topic', Kalman, queue_size=1)
    rate = rospy.Rate(10) # 10hz
    msg = Kalman()
    while not rospy.is_shutdown():
        msg.ax = ax
        msg.ay = ay
        msg.theta = theta
        msg.v = v
        pub.publish(msg)
        rate.sleep()
             
    rospy.spin() 

if __name__ == "__main__":
    try:
        paramTrans()
    except rospy.ROSInterruptException:
        pass
