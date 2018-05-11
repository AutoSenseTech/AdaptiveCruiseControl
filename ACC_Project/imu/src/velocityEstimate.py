#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import math
from imu.msg import Accelerator

def callback(data):

	theta = -0.10000
	alpha = -0.12500
	gamma = 1.43000

	x = data.linear_acceleration.x
	y = data.linear_acceleration.y
	z = data.linear_acceleration.z

	x_prime = x
	y_prime = -z*math.sin(theta) + y*math.cos(theta)
	z_prime = z*math.cos(theta) + y*math.sin(theta)

	x_pprime = x_prime * math.cos(alpha) + z_prime * math.sin(alpha)
	y_pprime = y_prime
	z_pprime = z_prime * math.cos(alpha) - x_prime * math.sin(alpha)

	x_ppprime = x_pprime * math.cos(gamma) + y_pprime * math.sin(gamma)
	y_ppprime = y_pprime * math.cos(gamma) - x_pprime * math.sin(gamma)
	z_ppprime = z_pprime
	
	msg = Accelerator()
	msg.x = x_ppprime
	msg.y = y_ppprime
	msg.z = z_ppprime
	pub.publish(msg)

if __name__ == '__main__':
	print("Initialize the velocity estimation node...")
	rospy.init_node('velocityEstimation', anonymous = True)
	pub = rospy.Publisher('accelerator', Accelerator, queue_size=1)
	sub = rospy.Subscriber('imu', Imu, callback)
	rospy.spin()
