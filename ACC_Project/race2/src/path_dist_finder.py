#!/usr/bin/env python

import rospy
import sys
import math
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from race2.msg import pid_input

x0 = 0
y0 = 0
x1 = 3.0
y1 = -1.0
xc = 0
yc = 0

yaw = 0

count = 0

pub = rospy.Publisher('error', pid_input, queue_size = 10)

#Gets yaw from a PoseWithCovarianceStamped message
def getYaw(pose_msg):
	quaternion = (
	  pose_msg.pose.pose.orientation.x,
	  pose_msg.pose.pose.orientation.y,
	  pose_msg.pose.pose.orientation.z,
	  pose_msg.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	yaw = euler[2]
	return yaw
	
#Calculates the swing angle given slope
def swing_calc(m, x0, y0, x1, y1):

	if x1 > x0:
		if y1 > y0:
			swing = math.atan(m)
		elif y1 == y0:
			swing = 0
		else:
			swing = -math.atan(math.fabs(m))
	elif x1 == x0:
		if y1 < y0:
			swing = -(math.pi) / 2
		elif y1 == y0:
			swing = 0 #probably should be an error instead
		else:
			swing = (math.pi) / 2
	else:
		if y1 < y0:
			swing = -(((math.pi) / 2) +
				 math.atan(math.fabs(1 / m)))
		elif y1 == y0:
			swing = math.pi
		else:
			swing = ((math.pi) / 2) + math.atan(math.fabs(1 / m))
	return swing

def poseCallback(cur_pose):
	global vel_input
	global x0
	global y0
	global x1
	global y1
	global xc
	global yc
	global yaw
	global count
	global pub
	global m
	global a
	global b
	global c
	global swing
	global error
	
	vel_input = rospy.get_param("velocity", 9.0)

	if count == 0:
		
		#Create a line connecting (x0, y0) and (x1, y1)
		#The line is in standard form (ax + by + c = 0)

		#Slope of line
		#Assume that x1 > x0 (car always moving forward)
		if(x1 != x0):
			rise = y1 - y0
			run = x1 - x0
			m = rise / run
		
		#Standard form coefficients
		a = y0 - y1
		b = x1 - x0
		c = (x0 * y1) - (x1 * y0)

		#Calculate angle between y-axis and formed line
		swing = swing_calc(m, x0, y0, x1, y1)
		count = count + 1
		error = 0

	else:

		#Get current position and yaw of car
		xc = cur_pose.pose.pose.position.x
		yc = cur_pose.pose.pose.position.y
		yaw = getYaw(cur_pose)

		#Calculate perpendicular distance from car to line
		ac = ((a * xc) + (b * yc) + c) / (math.sqrt((a**2 + b**2)))


		#Calculate heading angle relative to path using yaw and swing
		alpha = yaw - swing


		#Propagate the car into the future by 1 meter
		ad = 1.0
		error = ac + ad * math.sin(alpha)

		print("yaw: %4.5f\n"
		      "swing: %4.5f\n"
		      "alpha: %4.5f\n"
		      "ac: %4.5f\n"
		      "future error: %4.5f\n"
		       % (yaw, swing, alpha, ac, error))

	#Create, pack, and send message to control node
	msg = pid_input()
	msg.pid_error = error
	msg.pid_vel = vel_input
	pub.publish(msg)
	
	
#Callback method for initial point
#Assigns values to x0 and y0
#If the initial point changes, sets count to 0.
def initial_point_callback(init_msg):
	global x0
	global y0
	global count
	if init_msg.x != x0 or init_msg.y != y0:
		count = 0
	x0 = init_msg.x
	y0 = init_msg.y

#Callback method for finish point.
#Assigns values to x1 and y1.
def finish_point_callback(finish_msg):
	global x1
	global y1
	x1 = finish_msg.x
	y1 = finish_msg.y

#main
if __name__ == '__main__':
	print("Calculating distance from path...")
	rospy.init_node("path_dist_finder", anonymous = True)
	rospy.Subscriber("initial_point", Point, initial_point_callback)
	rospy.Subscriber("finish_point", Point, finish_point_callback)
	rospy.Subscriber("current_pose", PoseWithCovarianceStamped,
			  poseCallback)
	rospy.spin()
