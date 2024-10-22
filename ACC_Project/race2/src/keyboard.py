#!/usr/bin/env python
#This is the publisher node

import rospy
from race2.msg import drive_param # import the custom message
import curses
forward = 0;
offset = 7.25;
left = offset;
TIMEOUT = 0.1; #how long before a timeout
STEER = 1;
THROTTLE = 1;

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
rospy.init_node('keyboard_talker', anonymous=True)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)

stdscr.refresh()

key = ''
while key != ord('q'):
	key = stdscr.getch()
	stdscr.refresh()
        

        # fill in the conditions to increment/decrement throttle/steer

	if key == curses.KEY_UP:
            forward = forward + THROTTLE;
	    stdscr.addstr(2, 20, "Up  ")
	    stdscr.addstr(2, 25, '%.2f' % forward)
	    stdscr.addstr(5, 20, "    ")       
	elif key == curses.KEY_DOWN:
            forward = forward - THROTTLE;
	    stdscr.addstr(2, 20, "Down")
	    stdscr.addstr(2, 25, '%.2f' % forward)
	    stdscr.addstr(5, 20, "    ")
	if key == curses.KEY_LEFT:
            left = left + STEER;
	    stdscr.addstr(3, 20, "left")
	    stdscr.addstr(3, 25, '%.2f' % (left - offset))
	    stdscr.addstr(5, 20, "    ")
	elif key == curses.KEY_RIGHT:
            left = left - STEER;
	    stdscr.addstr(3, 20, "right")
	    stdscr.addstr(3, 25, '%.2f' % (left - offset))
	    stdscr.addstr(5, 20, "    ")
        elif key == curses.KEY_DC:
            # this key will center the steer and throttle
            forward = -80.0
	    left = offset
	    stdscr.addstr(2, 20, "Up  ")
	    stdscr.addstr(2, 25, '%.2f' % forward)
	    stdscr.addstr(5, 20, "    ") 
	    stdscr.addstr(3, 20, "right")
	    stdscr.addstr(3, 25, '%.2f' % left)
	    stdscr.addstr(5, 20, "    ")
	msg = drive_param()
	msg.velocity = forward
	msg.angle = left
	pub.publish(msg)

curses.endwin()
