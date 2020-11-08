#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped,Twist,PoseWithCovariance
from nav_msgs.msg import Path, Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from visualization_msgs.msg import Marker
import numpy as np
import math
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import roslib
import struct
import time
import rospkg
from nav_msgs.msg import Odometry,OccupancyGrid
from visualization_msgs.msg import MarkerArray,Marker
from geometry_msgs.msg import Twist
# from sensor_msgs import Joy


class Control(object):
    def __init__(self):
        self.sub_map = rospy.Subscriber("depthcam_map", OccupancyGrid, self.cb_map, queue_size=1)
        self.pub_marker_arr = rospy.Publisher("marker_arr", MarkerArray, queue_size=1)
        self.pub_testmap=rospy.Publisher("Omap", OccupancyGrid, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # sub joy to switch on or off
        self.auto = False
        # self.sub_joy = rospy.Subscriber('joy_teleop/joy', Joy, self.cb_joy, queue_size=1)

        self.scan_radius=[4,6,1] #[start,end,interval]
        self.check_map_array=[]
        for r in range(self.scan_radius[0],self.scan_radius[1],self.scan_radius[2]):
            for i in range(80):
                radius=r
                search_angle=i-40
                x = radius * math.cos(search_angle* math.pi / 180.0)
                y = radius * math.sin(search_angle* math.pi / 180.0)
                x_shift=x+1
                y_shift=y+5
                x_grid=x_shift*4
                y_grid=y_shift*4
                self.check_map_array.append(int(x_grid)+int(y_grid)*40)


        self.checking_Omap()
        print("init done")



    def cb_map(self,msg):
        get_map=msg.data
        left=None
        right=None

        for i in range(len(self.check_map_array)):
            if get_map[ self.check_map_array[i] ]>80:
                if left==None:
                    left=i
                else:
                    right=i

        if left!=None and right!=None:
            middle=(left+right)/2
            print('left=',left,',right=',right,',middle=',middle)


            vel=Twist()
            if(middle>45):
                print('Turn left')
                vel.linear.x=1
                vel.angular.z = 0.2
            elif(middle<35):
                print('Turn right')
                vel.linear.x=1
                vel.angular.z = -0.2
            else:
                print('Go straight')
                vel.linear.x=1

            if self.auto:
                self.pub_cmd_vel.publish(vel)

        self.pub_scan_curve()



    def pub_scan_curve(self):
        marker_arr=MarkerArray()
        for i in range(80):
            temp=Marker()
            temp.header.frame_id = "camera_link"
            temp.id = i
            radius=6
            search_angle=i-40
            temp.pose.position.x = radius * math.cos(search_angle* math.pi / 180.0)
            temp.pose.position.y = radius * math.sin(search_angle* math.pi / 180.0)
            temp.pose.orientation.w = 1.0
            temp.type = Marker.CUBE
            temp.action = Marker.ADD
            temp.color.r, temp.color.g, temp.color.b = (255, 0, 0)
            temp.color.a = 0.5
            temp.scale.x, temp.scale.y, temp.scale.z = (0.06, 0.06, 0.06)
            marker_arr.markers.append(temp)

        self.pub_marker_arr.publish(marker_arr)


    def checking_Omap(self):
        # Omap for checking whether it is the wall
        map = [0] * 40*40
        for i in range(len(self.check_map_array)):
            map[self.check_map_array[i]]=90

        Omap=OccupancyGrid()
    	Omap.header.frame_id = "camera_link"
    	Omap.info.resolution = 0.25
    	Omap.info.origin.position.x = -1
    	Omap.info.origin.position.y = -5
    	Omap.info.origin.position.z = 0
    	Omap.info.origin.orientation.w = 0
    	Omap.info.height = 40
    	Omap.info.width = 40
    	Omap.data =map
        self.pub_testmap.publish(Omap)




    def cb_joy(self, joy_msg):
        # MODE D
        start_button = 7
        back_button = 6
        # Start button
        if (joy_msg.buttons[start_button] == 1) and not self.auto:
            self.auto = True
            rospy.loginfo('go auto')
        elif joy_msg.buttons[back_button] == 1 and self.auto:
            self.auto = False
            rospy.loginfo('go manual')



if __name__ == "__main__":
    rospy.init_node("Control")
    rot=Control()
    rospy.spin()
