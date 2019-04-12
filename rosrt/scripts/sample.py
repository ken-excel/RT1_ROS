#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Wrench

rospy.init_node('tor_parameter')
tor_x = rospy.get_param('~tor_x', 15.0)
tor_rot = rospy.get_param('~tor_rot', 3.5)
pub = rospy.Publisher('/mobile_base/commands/torque', Wrench, queue_size=10)

r = rospy.Rate(10.0)
for i in range(20):
	r.sleep()

tor = Wrench()
tor.torque.z = tor_rot
r = rospy.Rate(10.0)
for i in range(50):
	pub.publish(tor)
	r.sleep()

tor = Wrench()
r = rospy.Rate(10.0)
for i in range(10):
	pub.publish(tor)
	r.sleep()

tor = Wrench()
tor.torque.z = -tor_rot
r = rospy.Rate(10.0)
for i in range(50):
	pub.publish(tor)
	r.sleep()

tor = Wrench()
r = rospy.Rate(10.0)
for i in range(10):
	pub.publish(tor)
	r.sleep()

