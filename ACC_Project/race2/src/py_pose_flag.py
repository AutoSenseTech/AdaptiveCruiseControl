#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseWithCovarianceStamped

#The 'stop' boolean
stop = 0

#stop publisher
pub = rospy.Publisher('py_pose_flag', Int16, queue_size = 10)

def poseCallback(msg):
	global stop
	x_stop_min = 2.8
	x_stop_max = 3.2
	y_stop_min = -1.3
	y_stop_max = -0.7
	if(msg.pose.pose.position.x > x_stop_min
	   and msg.pose.pose.position.x < x_stop_max
	   and msg.pose.pose.position.y > y_stop_min
	   and msg.pose.pose.position.y < y_stop_max):
		stop = 1
	print str(stop)
	new_msg = Int16()
	new_msg.data = stop
	pub.publish(new_msg)

if __name__ == '__main__':
	rospy.init_node("py_pose_flag", anonymous = True)
	rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, poseCallback)
	rospy.spin()
