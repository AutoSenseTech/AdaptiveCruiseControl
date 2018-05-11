#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import math
from race2.msg import drive_param

dataNum = 200 # Define the number of the data to calibrate the IMU
counter = 0
sumLinearAcceleration = [0, 0, 0, 0, 0, 0]



def callback(data):
	global counter
	global dataNum
	global sumLinearAcceleration
        
	msg = drive_param()
	if counter < dataNum:
		sumLinearAcceleration[0] += data.linear_acceleration.x
		sumLinearAcceleration[1] += data.linear_acceleration.y
		sumLinearAcceleration[2] += data.linear_acceleration.z
		counter += 1
	
	if counter == dataNum:
		sub.unregister()
		raw_input("The car will run forward and please put the car in a empty space, press Enter to start...")
		counter += 1
        	msg.velocity = 8.0
        	msg.angle = 7.25
        	pub.publish(msg)
		rospy.Subscriber("imu", Imu, callback)

	if (counter > dataNum and counter < 2*dataNum + 1):
	    	sumLinearAcceleration[3] += data.linear_acceleration.x
		sumLinearAcceleration[4] += data.linear_acceleration.y
		sumLinearAcceleration[5] += data.linear_acceleration.z
		counter +=1


	if counter == 2 * dataNum + 1:
        	msg.velocity = 0.0
        	msg.angle = 7.25
        	pub.publish(msg)
		meanLinearAcceleration = [x / dataNum \
								for x in sumLinearAcceleration]
	
		x = meanLinearAcceleration[0]
		y = meanLinearAcceleration[1]
		z = meanLinearAcceleration[2]
		theta = math.atan(y/z)
		print("theta = ",theta)
		x_prime = x
		y_prime = -z*math.sin(theta) + y*math.cos(theta)
		z_prime = z*math.cos(theta) + y*math.sin(theta)
		alpha = math.atan(-x_prime / z_prime)
		print("alpha = ",alpha)
		x_pprime = x_prime * math.cos(alpha) + z_prime * math.sin(alpha)
		y_pprime = y_prime
		z_pprime = z_prime * math.cos(alpha) - x_prime * math.sin(alpha)

		x1 = meanLinearAcceleration[3]
		y1 = meanLinearAcceleration[4]
		z1 = meanLinearAcceleration[5]
		x1_prime = x1
		y1_prime = -z1*math.sin(theta) + y1*math.cos(theta)
		z1_prime = z1*math.cos(theta) + y1*math.sin(theta)
		x1_pprime = x1_prime * math.cos(alpha) + z1_prime * math.sin(alpha)
		y1_pprime = y1_prime
		z1_pprime = z1_prime * math.cos(alpha) - x1_prime * math.sin(alpha)
		
		print("x_pp", x1_pprime)
		print("y_pp", y1_pprime)
		print("z_pp", z1_pprime)
		gamma = math.atan(y1_pprime / x1_pprime)
		x1_ppprime = x1_pprime * math.cos(gamma) + y1_pprime * math.sin(gamma)
		y1_ppprime = y1_pprime * math.cos(gamma) - x1_pprime * math.sin(gamma)
		z1_ppprime = z1_pprime
		print("gamma = ", gamma)
		print("x = ", x1_ppprime)
		print("y = ", y1_ppprime)
		print("z = ", z1_ppprime)
		counter += 1
	
	

if __name__ == '__main__':
	print("IMU is Calibrating")
	raw_input("Press Enter when the four wheels standing on the plane to continue...")	
	rospy.init_node('imu_calibrater', anonymous = True)
    	pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
	sub = rospy.Subscriber("imu", Imu, callback)
	rospy.spin()
	
