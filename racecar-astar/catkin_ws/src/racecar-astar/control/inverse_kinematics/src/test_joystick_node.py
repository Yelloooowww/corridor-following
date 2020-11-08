#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy

class test_joystick_node(object):
	def __init__(self):

		# ROS subscriber and publisher
		self.sub = rospy.Subscriber("/joy", Joy, self.JoyCallback, queue_size = 1)
		self.pub = rospy.Publisher("/car_cmd", TwistStamped, queue_size=1)
                self.r = rospy.Rate(10)
		self.protect_flag = True
	def JoyCallback(self, joy_msg):
		car_cmd = TwistStamped()

		if joy_msg.buttons[7] == 1:
			car_cmd.twist.linear.x = 0
			car_cmd.twist.angular.z = 0
			self.pub.publish(car_cmd)
			self.r.sleep()
			self.protect_flag = not self.protect_flag
			print "emergency stop"
		elif self.protect_flag==True :
			car_cmd.twist.linear.x = joy_msg.axes[1]
			car_cmd.twist.angular.z = joy_msg.axes[3]*0.785
			self.pub.publish(car_cmd)
			self.r.sleep()
		else:
			car_cmd.twist.linear.x = 0
			car_cmd.twist.angular.z = 0
			self.pub.publish(car_cmd)
			self.r.sleep()
		

if __name__ == '__main__':
	# start ros node
	rospy.init_node('test_joystick_node', anonymous=True)
	ob = test_joystick_node()
	rospy.spin()
