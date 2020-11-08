#!/usr/bin/env python
import rospy
from std_msgs.msg import Twist2DStamped
from roboteq_controller.msg import hdc2460_msgs, motorsCmd
from roboteq_controller.srv import estop_srv, estop_srvResponse

class inverse_kinematics_node(object):
	def __init__(self):

		# parameters
		self.rotor_dist=2.44

		# ROS subscriber and publisher
		self.sub_wamv_cmd = rospy.Subscriber("/car_cmd", Twist2DStamped, self.wamv_cmd_callback)
		self.pub_wamv_cmd = rospy.Publisher("/motor_cmd", motorsCmd, queue_size=1)


	def wamv_cmd_callback(self, msg_wamv_cmd):
		# read message
		v = msg_wamv_cmd.v
		omega = msg_car_cmd.omega
		
		# create motorscmd message instance
		c = motorsCmd()

		# differential drive assumption
		c.vel_left = (2*v-omega*self.rotor_dist)/2
		c.vel_right = (2*v+omega*self.rotor_dist)/2
		self.pub_wamv_cmd.publish(c)

if __name__ == '__main__':
	# start ros node
	rospy.init_node('inverse_kinematics_node', anonymous=True)
	ob = inverse_kinematics_node()