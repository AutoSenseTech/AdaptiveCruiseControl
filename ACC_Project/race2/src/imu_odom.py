#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

prev_pos = [0, 0, 0]
prev_vel = [0, 0, 0]
prev_acc = [0, 0, 0]
cur_pos = [0, 0, 0]
cur_vel = [0, 0, 0]
cur_acc = [0, 0, 0]

prev_time = 0
cur_time = 0

pub = rospy.Publisher("odom", Odometry, queue_size = 1)

def imu_callback(msg):
	global prev_pos
	global prev_vel
	global prev_acc
	global cur_pos
	global cur_vel
	global cur_acc
	global prev_time
	global cur_time
	global pub
	
	cur_acc[0] = msg.linear_acceleration.x - 1.0
	cur_acc[1] = msg.linear_acceleration.y
	#cur_acc[2] = msg.linear_acceleration.z

	prev_time = msg.header.stamp
	cur_time = rospy.get_rostime()
	dt = cur_time.secs - prev_time.secs

	cur_vel[0] = prev_vel[0] + (cur_acc[0] * dt)
	cur_vel[1] = prev_vel[1] + (cur_acc[1] * dt)
	#cur_vel[2] = prev_vel[2] + (cur_acc[2] * dt)

	cur_pos[0] = prev_pos[0] + (cur_vel[0] * dt)
	cur_pos[1] = prev_pos[1] + (cur_vel[1] * dt)
	#cur_pos[2] = cur_pos[2] + (cur_vel[2] * dt)
	
	odom_msg = Odometry()
	odom_msg.header.stamp = msg.header.stamp
	odom_msg.header.frame_id = "odom"
	odom_msg.child_frame_id = "base_imu_link"
	
	odom_msg.pose.pose.position.x = cur_pos[0]
	odom_msg.pose.pose.position.y = cur_pos[1]
	odom_msg.pose.pose.position.z = cur_pos[2]
	odom_msg.pose.pose.orientation = msg.orientation
	#odom_msg.pose.covariance = msg.orientation_covariance #need to resize

	odom_msg.twist.twist.linear.x = cur_vel[0]
	odom_msg.twist.twist.linear.y = cur_vel[1]
	odom_msg.twist.twist.linear.z = cur_vel[2]
	odom_msg.twist.twist.angular = msg.angular_velocity
	#odom_msg.twist.covariance = msg.angular_velocity_covariance #need to resize
	
	pub.publish(odom_msg)

	prev_pos = cur_pos
	prev_vel = cur_vel
	prev_acc = cur_acc
	
if __name__ == '__main__':
	print("Listening for IMU messages...")
	rospy.init_node('imu_odom', anonymous = True)
	rospy.Subscriber("imu", Imu, imu_callback)
	rospy.spin()
