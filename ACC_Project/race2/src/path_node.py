#!/usr/bin/env python

#imports
import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Int16

#Variables to initialize on node startup.
i = 0
#Path for one turn (map = fgh_rasl_2)
#path = [[0, 0], [1, 0], [2, 0], [3, 0], [4, 0], [4.75, -1], [4.9, -4], [4.9, -6]]

#Path for two turns (map = two_turns)
path = [[0, 0], [0.5, 0], [1.6, -1.5], [1.65, -6.0], [1.2, -6.5], [0.1, -6.9]]

path_length = len(path) - 2
stop = 0
stop_pub = rospy.Publisher('path_stop', Int16, queue_size = 10)
init_point_pub = rospy.Publisher('initial_point', Point, queue_size = 10)
finish_point_pub = rospy.Publisher('finish_point', Point, queue_size = 10)
current_pose_pub = rospy.Publisher('current_pose', PoseWithCovarianceStamped,
				    queue_size = 10)

#Given a point and a publisher of Point messages, fills the message and
#publishes it.
def point_message_helper(point, pub):
	point_msg = Point()
	point_msg.x = point[0]
	point_msg.y = point[1]
	pub.publish(point_msg)

#Given a pose and a publisher of PoseWithCovarianceStamped() messages,
#fills the message and publishes it.
def pose_message_helper(pose, pub):
	pose_msg = PoseWithCovarianceStamped()
	pose_msg = pose
	pub.publish(pose_msg)

#Given an integer and a publisher of Int16() messages,
#fills the message and publishes it.
def stop_message_helper(stop, pub):
	stop_msg = Int16()
	stop_msg.data = stop
	stop_pub.publish(stop_msg)

#Given a PoseWithCovarianceStamped message, returns the x-coordinate.
def get_x_position(pose_msg):
	x = pose_msg.pose.pose.position.x
	return x

#Given a PoseWithCovarianceStamped message, returns the y-coordinate.
def get_y_position(pose_msg):
	y = pose_msg.pose.pose.position.y
	return y
	
#When a message arrives on the amcl_pose topic, determine if the car has
#reached the next point on the path. If it has, make sure it keeps going.
#If the car has reached the end of the path, the car stops.
def pose_callback(msg):
	global i
	global path
	global p0
	global p1
	global stop
	global hit_box_size
	
	#On the first traversal, assign p0 and p1 and get hitbox size.
	#if i == 0:
		#p0 = path[i]
		#p1 = path[i + 1]
		#hit_box_size = rospy.get_param("/py_pose_flag/hit_box_size",
		#				 0.5)

	#Get the current x and y positions.
	if stop == 0:
		x_cur = get_x_position(msg)
		y_cur = get_y_position(msg)

		#If the car has reached p1, do one of two things:
		#	1. Stop the car if it has reached the final point in the path.
		#	2. Assign new values to the starting and finishing points
		#	   if the path still has points remaining.
		if (x_cur > p1[0] - hit_box_size and x_cur < p1[0] + hit_box_size and
	    	y_cur > p1[1] - hit_box_size and y_cur < p1[1] + hit_box_size):
			i = i + 1
			if i > path_length:
				stop = 1
				stop_message_helper(stop, stop_pub)
			else:	
				p0 = path[i]
				p1 = path[i + 1]

		#Fill and publish point messages and pose message
		point_message_helper(p0, init_point_pub)
		point_message_helper(p1, finish_point_pub)
		pose_message_helper(msg, current_pose_pub)
	

		#print for debugging purposes
		print("p0x: %4.5f\n"
	      	      "p0y: %4.5f\n"
	      	      "p1x: %4.5f\n"
	      	      "p1y: %4.5f\n"
	      	      "stop: %4.5f\n"
	      	      "i: %4.5f\n" % (p0[0], p0[1], p1[0], p1[1], stop, i))
	
	else:
		print("The car has stopped.")
		
#Subscribes to PoseWithCovarianceStamped messages on the amcl_pose topic.
#pose_callback is invoked on arrival of messages.
def pose_tracker():
	global stop
	global path
	global p0
	global p1
	global hit_box_size
	p0 = path[0]
	p1 = path[1]
	hit_box_size = rospy.get_param("/path_node/hit_box_size", 0.5)
	rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped,
			 pose_callback)
	rospy.spin()
 
#main
if __name__ == '__main__':
	rospy.init_node('path_node', anonymous = True)
	pose_tracker()
