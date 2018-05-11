#!/usr/bin/env python

import rospy
import sys
import math
from sensor_msgs.msg import LaserScan
from race2.msg import pid_input
from std_msgs.msg import Int16

# Velocity of car, gets converted to PWM in talker node
vel_input = rospy.get_param("velocity", 9.0)

# Theta input, gives the angular separation of chosen LIDAR measurements
theta_input =  rospy.get_param("theta", 30.0)

# How far the car will stay away from the wall
desired_trajectory = rospy.get_param("trajectory", 1.0)

# Error publisher, picked up by control node
pub = rospy.Publisher('error', pid_input, queue_size=10)

##Input: 
##	data: Lidar scan data (ranges in meter)
##	theta: The angle (in radians) at which the distance is requried
##Output: 
##	distance of scan at angle theta

def getRange(data,theta):

# Find the index of the arary that corresponds to angle theta.
# Return the lidar scan value at that index
# Do some error checking for NaN and absurd values
## Your code goes here
	
	#is theta NaN?
	x = float(theta)
	if math.isnan(x):
		print('theta is not a number: %s', theta)
		raise TypeError('The given value %s is not a number', theta)
	
	#is theta absurd?
	if theta < data.angle_min or theta > data.angle_max:
		print 'Given theta is not within angle range\ntheta=%d\nmin=%4.2f\nmax=%4.2f' %(theta, data.angle_min, data.angle_max)
		raise ValueError('Theta is not within range')

	##theta is a number; find index for given theta
	step = 0
	
	##angle_min = -2.36
	angle_total = data.angle_min
	while angle_total < theta:
		step = step + 1
		angle_total = angle_total + data.angle_increment
	
	##found index corresponding to theta
	return data.ranges[step]


def callback(data):
	global theta_input
	global desired_trajectory
	global vel_input
	global pub

	print '%.2f %.2f' % (theta_input, vel_input)
	theta_input_rad = math.radians(theta_input)
	theta_offset = math.radians(-90) #offset to 0
	swing = theta_input_rad + theta_offset #theta to radians
	a = getRange(data, swing)
	b = getRange(data, theta_offset)
	print 'a: %4.5f\nb: %4.5f' % (a, b)
	
	
	## Your code goes here (actually it's this entire method)
	top = a * math.cos(theta_input_rad) - b
	bot = a * math.sin(theta_input_rad)

	# Heading angle
	alpha = math.atan(float(top)/bot)

	# How far car is from wall
	ab = b * math.cos(alpha)

	#ac = velocity of car (m/s) * duration of future travel (m)
	#ac is really just how far to future project the car in a straight line
	ac = 2.36 * 0.5
	
	# How far car will be from wall in 0.5 seconds if continuing on same path
	cd = ab + ac*math.sin(alpha)

	print 'alpha: %4.5f\nab: %4.5f\nac: %4.5f\ncd: %4.5f\n' % (alpha,ab,ac,cd)

	# How far car will be from wall in 0.5 seconds if continuing on same path
	error = cd - desired_trajectory
	
	# Create, pack, and send message to control node
	msg = pid_input()
	msg.pid_error = error
	msg.pid_vel = vel_input 
	pub.publish(msg) 
	
def flagCallback(flag_msg):
	global vel_input
        if flag_msg.data == 1:
		vel_input = 0.0
	else:
		vel_input = 9.0
	
	

if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("pose_flag", Int16, flagCallback)
	rospy.Subscriber("scan", LaserScan, callback)
	rospy.spin()
