#!/usr/bin/env python
import rospy
import numpy as np
import random
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,Point32
import time
from std_msgs.msg import String




        
    
    



map_point_cloud = [(0.0,0.0)]*2
pcl_list = []
map_list = []

def PT_callback(data):
    global map_list
    # pcl_list = [i for i in data[1:-1].split(",")] 
    n = 10
    x = math.floor(data.x*n)/n
    y = math.floor(data.y*n)/n
    point = (x, y)
    s = set(map_list)
    if point not in s:
        bkpoint = bk2Point(point)
        wall.points.append(bkpoint)
    rate = rospy.Rate(60)
    rate.sleep()

def bk2Point(data):
    p = Point()
    p.x = float(data[0])
    p.y = float(data[1])
    return p  
   






def ransac():
    global pcl_list, wall
    rospy.init_node('test')
    rospy.Subscriber('/points_data', Point, PT_callback)
    wall_pub = rospy.Publisher('/test', Marker,queue_size=10)


    wall = Marker()
    wall.header.frame_id = '/base_link'
    wall.type = Marker.POINTS
    wall.scale.x = 0.05
    wall.scale.y = 0.05
    wall.color.b = 1.0
    wall.color.g = 1.0
    wall.color.a = 10.0



    while not rospy.is_shutdown():
        
        

        wall_pub.publish(wall)

        
    rospy.spin()



if __name__== '__main__':
    try:
        ransac()

    except rospy.ROSInterruptException:
        pass

#TPEMIST/2021OCT