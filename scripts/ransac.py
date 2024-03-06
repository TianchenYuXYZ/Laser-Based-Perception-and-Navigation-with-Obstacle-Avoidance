#!/usr/bin/env python
import rospy
import random
import math
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as ptc2
from geometry_msgs.msg import Point,Point32
import time
from std_msgs.msg import String
from copy import deepcopy
import numpy as np


def find_A_B(point1, point2):
    x_1 = point1[0]
    y_1 = point1[1]
    x_2 = point2[0]
    y_2 = point2[1]

    if x_1==x_2:
        a = -1
    else:
        a = (y_2 -y_1) / (x_2-x_1)

    b = y_1 -a * x_1

    return a,b

def make_walls(points,max_try = 1500,bias =0.75):

    wall.points = []

    while len(points)>30:
        # global wall
        inner_total = 0
    
        for i in range(max_try):

            if len(points) < 2:
                    continue
            
            sample = random.sample(range(len(points)),2)
            a,b = find_A_B(points[sample[0]],points[sample[1]])

            inliner = 0
            inner_list = []
            for i in range(len(points)):
                y_a = a * points[i][0] + b
                sigma = y_a - points[i][1]
                if abs(sigma) < bias:
                    inliner += 1
                    inner_list.append(points[i])
            

            if inliner > inner_total:
                inner_total = inliner
                min_p = inner_list[0] #best_point 1
                max_p = inner_list[-1] #best_point 2

        wall.points.append(bk2Point(min_p))
        wall.points.append(bk2Point(max_p))
        
        storage_list=[]
        for i in range(len(points)):
            aa,bb = find_A_B(min_p,max_p)
            if i not in range(len(points)):
                continue
            y_aa = aa * points[i][0] + bb
            sigma = y_aa - points[i][1]
            if abs(sigma) > bias:
                storage_list.append(points[i])
        
        points = storage_list
        
    # print(len(wall.points))
    wall_pub.publish(wall)

def find_wall(points, bias=1, max_try = 1000): #(points,line&point's bias,)

    inner_total = 0
    
    # best_point1 = (0.0,0.0)
    # best_point2 = (0.0,0.0)
    wall.points =[]
    
    
    for i in range(max_try):

        if len(points) < 2:
                continue
        sample = random.sample(range(len(points)),2)

        a = find_A_B(points[sample[0]],points[sample[1]])[0]
        b = find_A_B(points[sample[0]],points[sample[1]])[1]

        inliner = 0
        for i in range(len(points)):
            y_a = a * points[i][0] + b
            sigma = y_a - points[i][1]
            if abs(sigma) < bias:
                inliner += 1

        if inliner > inner_total:
            inner_total = inliner
            best_point1 = points[sample[0]]
            best_point2 = points[sample[1]]
    # print(best_point1,best_point2)
 
    wall.points.append(bk2Point(best_point1))
    wall.points.append(bk2Point(best_point2))
    wall_pub.publish(wall)
    # print(len(wall.points))


    
    

pcl_list=[]

def PT_callback(data):
    global pcl_list,PL_list
    point_gen_list = ptc2.read_points_list(data)
    pcl_list = [(0.0,0.0)]*(len(point_gen_list))

    for i in range(len(point_gen_list)):
        n = 10^1
        pcl_list[i] = (math.floor(point_gen_list[i].x*n)/n,math.floor(point_gen_list[i].y*n)/n)

def bk2Point(data):
    p = Point()
    p.x = float(data[0])
    p.y = float(data[1])
    return p

def RANSAC():
    global best_point1, best_point2, pcl_list, wall_pub, wall,PL_list
    rospy.Subscriber('/point_cloud2', PointCloud2, PT_callback)
    wall_pub = rospy.Publisher('/ransac_wall', Marker)


    wall = Marker()
    wall.header.frame_id = '/base_link'
    wall.type = Marker.LINE_LIST
    wall.scale.x = 0.05
    wall.scale.y = 0.05
    wall.color.b = 1.0
    wall.color.g = 1.0
    wall.color.a = 10.0

    while not rospy.is_shutdown():
        if len(pcl_list) < 2:
            continue  
        make_walls(pcl_list)
        #find_wall(pcl_list)
        
    rospy.spin()



if __name__== '__main__':
    try:
        rospy.init_node('ransac')
        RANSAC()

    except rospy.ROSInterruptException:
        pass

#TPEMIST/2021OCT