#!/usr/bin/env python
##This file subscribes to drive_parameters topic, and then publishes on the drive_ackermann topic, which uses ackermann_msgs

import rospy
from race2.msg import drive_param
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

given_speed = 0 ## m/s
given_steering_angle = 0  ## radians
pub = rospy.Publisher('drive_ackermann', AckermannDriveStamped, queue_size=10)

##need to handle the kill command???

##takes place of the actual conversion
def mapValue(value, key_min, key_max, a_min, a_max):
	if(not(value > key_max or value < key_min)):
		key_span = key_max - key_min
		a_span = a_max - a_min
		val_scale = (float(a_span)/key_span)
		
		return (value - key_min)*val_scale + a_min
	raise ValueError('The given value was not in range')

def callback(data):
  print "velocity=%.2f  angle=%.2f" % (data.velocity, data.angle)
  given_speed = mapValue(data.velocity, 0, 100, 0, 40) ##can only go in positive direction
  given_steering_angle = (data.angle, -100, 100, -0.7, 0.7) ##assuming range of servo is +-.7 radians
  msg = AckermannDriveStamped()
  msg.drive.speed = given_speed
  msg.drive.steering_angle = given_steering_angle
  pub.publish(msg)

## subscribe to drive_parameters topic
def converter():
  rospy.init_node('converter', anonymous=True)
  rospy.Subscriber('drive_parameters', drive_param, callback)

  # won't exit until node is stopped
  rospy.spin()

if __name__ == '__main__':
    print ("converter started")
    converter()
  
