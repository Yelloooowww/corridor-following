#!/usr/bin/env python
import rospy
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid


class map_processing(object):
    def __init__(self):
        self.sub_map = rospy.Subscriber("depthcam_map", OccupancyGrid, self.cb_map, queue_size=1)
        self.pub_testmap=rospy.Publisher("Omap", OccupancyGrid, queue_size=1)

    def cb_map(self,msg):
        Omap=OccupancyGrid()
        Omap.header=msg.header
        Omap.info=msg.info
        tmp_map=[0]*2500

        for i in range(2500):
            tmp_map[i]=msg.data[i]
            if i%50 <25:
                tmp_map[i]=100


        for y in range(25):
            x=y / math.atan(math.radians(40))
            for i in range(2500):
                if i<1250:
                    for xx in range( int(x)+24 ):
                        tmp_map[50* abs(y-25)+xx]=100
                else:
                    for xx in range( int(x)+24 ):
                        tmp_map[50* abs(y+24)+xx]=100


        Omap.data=tmp_map
        self.pub_testmap.publish(Omap)
        # print('Pub Omap')


if __name__ == "__main__":
    rospy.init_node("map_processing")
    rot=map_processing()
    rospy.spin()
