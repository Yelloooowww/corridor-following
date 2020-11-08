#!/usr/bin/env python

import rospy
from pure_pursuit import PurePursuit
from astar.srv import GoToPos, GoToPosResponse, GoToPosRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from nav_msgs.msg import Path, Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from anchor_measure.msg import PoseDirectional
from visualization_msgs.msg import Marker
from subt_msgs.msg import ArtifactPoseArray, ArtifactPose
import tf
import numpy as np
import math


class Navigation(object):
    def __init__(self):
        rospy.loginfo("Initializing %s" % rospy.get_name())

        ## parameters
        self.map_frame = rospy.get_param("~map_frame", 'odom')
        self.bot_frame = None
        self.switch_thred = 1 # change to next lane if reach next

        self.pursuit = PurePursuit()
        self.pursuit.set_look_ahead_distance(0.1)

        ## node path
        while not rospy.has_param("/guid_path"):
            rospy.loginfo("Wait for /guid_path")
            rospy.sleep(0.5)
        self.guid_path = rospy.get_param("/guid_path")
        self.last_node = -1
        self.all_anchor_ids = rospy.get_param("/all_anchor_ids")
        self.joy_lock = False

        ## set first tracking lane
        self.set_lane(True)

        # variable
        self.target_global = None
        self.listener = tf.TransformListener()

        # markers
        self.pub_goal_marker = rospy.Publisher(
            "goal_marker", Marker, queue_size=1)

        self.pub_target_marker = rospy.Publisher(
            "target_marker", Marker, queue_size=1)

        # publisher, subscriber
        self.pub_pid_goal = rospy.Publisher(
            "pid_control/goal", PoseStamped, queue_size=1)

        self.req_path_srv = rospy.ServiceProxy("plan_service", GetPlan)

        self.sub_box = rospy.Subscriber(
            "anchor_pose", PoseDirectional, self.cb_goal, queue_size=1)
        
        sub_joy = rospy.Subscriber('joy_teleop/joy', Joy, self.cb_joy, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.tracking)

        # print("init done")

    def tracking(self, event):
        # print("tracking loop")

        if self.target_global is None:
            # rospy.logerr("%s : no goal" % rospy.get_name())
            return
        else:
            rospy.loginfo("%s :have seen goal" % rospy.get_name())   

        end_p = self.transform_pose(
            self.target_global.pose, self.bot_frame, self.map_frame)

        self.pub_target_marker.publish(self.to_marker(end_p, [0, 0, 1]))

        d = math.sqrt(end_p.pose.position.x**2 + end_p.pose.position.y**2)
        if d <= self.switch_thred:
            rospy.logwarn("goal reached, to next goal")
            self.target_global = None
            self.set_lane(True)
            return

        start_p = PoseStamped()
        start_p.pose.position.x = 1
        start_p.pose.position.y = 0
        start_p.pose.position.z = 0

        req_path = GetPlanRequest()
        req_path.start = start_p
        req_path.goal = end_p

        try:
            res_path = self.req_path_srv(req_path)
        except:
            rospy.logerr("%s : path request fail" % rospy.get_name())
            return

        self.pursuit.set_path(res_path.plan)

        goal = self.pursuit.get_goal(start_p)
        if goal is None:
            rospy.logwarn("goal reached, to next goal")
            self.target_global = None
            self.set_lane(True)
            return

        goal = self.transform_pose(goal.pose, self.map_frame, self.bot_frame)
        goal.header.frame_id = self.map_frame

        self.pub_goal_marker.publish(self.to_marker(goal))

        self.pub_pid_goal.publish(goal)

    def cb_goal(self, msg):

        self.target_global = msg.pose # posestamped
        self.bot_frame = self.target_global.header.frame_id
        self.target_global.pose = self.transform_pose(msg.pose.pose, self.map_frame, msg.header.frame)
        self.target_global.header.frame_id = self.map_frame
    
    def cb_joy(self, msg):

        switch = msg.axes[-1]
        if switch == 0:
            self.joy_lock = False

        if self.joy_lock:
            return
        
        switch = msg.axes[-1]
        if switch == 1:
            rospy.loginfo("Joy to the next lane.")
            self.set_lane(True)
            self.joy_lock = True
        elif switch == -1:
            rospy.loginfo("Joy to the last lane.")
            self.set_lane(False)
            self.joy_lock = True
    
    def set_lane(self, next):

        # to next node
        if next:
            self.last_node += 1
        else:
            self.last_node -= 1
        if self.last_node >= len(self.guid_path)-1:
            rospy.loginfo("It's the last lane.")
            self.last_node -= 1
            return
        elif self.last_node < 0:
            rospy.loginfo("Back to first lane.")
            self.last_node = 0

        # set last, next node (anchor)
        rospy.set_param("/guid_lane_last", self.all_anchor_ids[self.guid_path[self.last_node]])
        rospy.set_param("/guid_lane_next", self.all_anchor_ids[self.guid_path[self.last_node+1]])

        # set pozyx ranging tag id
        try:
            rospy.delete_param("/anchorballs")
        except KeyError:
            rospy.logerr("No such param /anchorballs")
        pozyx_id = {}
        pozyx_id[self.guid_path[self.last_node]] = self.all_anchor_ids[self.guid_path[self.last_node]]
        pozyx_id[self.guid_path[self.last_node+1]] = self.all_anchor_ids[self.guid_path[self.last_node+1]]
        rospy.set_param("/anchorballs", pozyx_id)
    
    def to_marker(self, goal, color=[0, 1, 0]):
        marker = Marker()
        marker.header.frame_id = goal.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.orientation.w = 1
        marker.pose.position.x = goal.pose.position.x
        marker.pose.position.y = goal.pose.position.y
        marker.id = 0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        return marker

    def transform_pose(self, pose, target_frame, source_frame):
        try:
            (trans_c, rot_c) = self.listener.lookupTransform(
                target_frame, source_frame, rospy.Time(0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("faile to catch tf %s 2 %s" %
                         (target_frame, source_frame))
            return

        trans_mat = tf.transformations.translation_matrix(trans_c)
        rot_mat = tf.transformations.quaternion_matrix(rot_c)
        tran_mat = np.dot(trans_mat, rot_mat)

        # print(trans_c)

        target_mat = np.array([[1, 0, 0, pose.position.x],
                               [0, 1, 0, pose.position.y],
                               [0, 0, 1, pose.position.z],
                               [0, 0, 0, 1]])
        target = np.dot(tran_mat, target_mat)
        quat = tf.transformations.quaternion_from_matrix(target)
        trans = tf.transformations.translation_from_matrix(target)

        t_pose = PoseStamped()
        t_pose.header.frame_id = target_frame
        t_pose.pose.position.x = trans[0]
        t_pose.pose.position.y = trans[1]
        t_pose.pose.position.z = trans[2]
        t_pose.pose.orientation.x = quat[0]
        t_pose.pose.orientation.y = quat[1]
        t_pose.pose.orientation.z = quat[2]
        t_pose.pose.orientation.w = quat[3]

        return t_pose


if __name__ == "__main__":
    rospy.init_node("navigation_guid")
    nav = Navigation()
    rospy.spin()
