#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
import sys, tty, termios


msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
f = forward
b = back
r = to the right (CW)
l = to the left (CCW)

After specifying a direction, input the distance (m) or turn (rad)

Enter q to quit
"""

directionBindings = {
		'f':(0.3, 0),
		'b':(-0.3, 0),
		'r':(0, -1),
		'l':(0, 1)
			}


def getGoalTime(direction, distance):
	""" Return the time to run, assuming forward speed = 0.3m/s and angular speed = 1 rad/s"""
	if direction == 'f' or direction == 'b':
		return (distance/0.3)
	elif direction == 'l' or direction == 'r':
		return distance
	else:
		return 0


if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('/cmd_vel', Twist)
	rospy.init_node('schneider_teleop')

	twist_go = Twist()
	twist_go.linear.x = 0.3; twist_go.linear.y = 0; twist_go.linear.z = 0
	twist_go.angular.x = 0; twist_go.angular.y = 0; twist_go.angular.z = 1.0
	twist_stop = Twist()
	twist_stop.linear.x = 0; twist_stop.linear.y = 0; twist_stop.linear.z = 0
	twist_stop.angular.x = 0; twist_stop.angular.y = 0; twist_stop.angular.z = 0

	try:
		rospy.loginfo(msg)

		while (1):
			pub.publish(twist_stop)
			key = raw_input("Would you like to go (f) forward, (b)ack, (r)ight [CW], or (l)eft [CCW]? ")
			if key == 'q':
				pub.publish(twist_stop)
				break
			if key in directionBindings.keys():
				cmd = raw_input("Enter a distance (m) or angle (rad) to traverse: ")
				if cmd == 'q':
					pub.publish(twist_stop)
					break
				try:
					cmd_num = float(cmd)
				except ValueError:
					rospy.logwarn("You gave me a message of non-float type. I want a float!")
					cmd_num = 0

				start_time = rospy.get_time()
				goal_time = getGoalTime(key, cmd_num)

				twist_go.linear.x = directionBindings[key][0]
				twist_go.angular.z = directionBindings[key][1]
				pub.publish(twist_go)
				while not rospy.is_shutdown() and (rospy.get_time()-start_time) < goal_time:
					rospy.loginfo("Going %s, at time %f/%f", key, rospy.get_time()-start_time, goal_time)
					rospy.sleep(0.05)
				pub.publish(twist_stop)

	except:
		rospy.logwarn(sys.exc_info()[0])

	finally:
		pub.publish(twist_stop)
    	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)