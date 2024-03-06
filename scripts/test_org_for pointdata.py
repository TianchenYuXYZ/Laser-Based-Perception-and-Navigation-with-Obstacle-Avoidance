#!/usr/bin/env python
import rospy
import numpy as np
import random
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,Point32
import time
from std_msgs.msg import String


def find_wall(len_p, points, bias=1, max_try = 1000, error_rate = 0.70): #(points,line&point's bias,)
    global best_point1, best_point2, ban_list

    best_point1 = (0.0,0.0)
    best_point2 = (0.0,0.0)
    inner_total = 0
    ban_list = []

    for i in range(max_try):
        if len(points) < 2:
            continue
        sample = random.sample(range(len(points)),2)
        x_1 = points[sample[0]][0]
        y_1 = points[sample[0]][1]
        x_2 = points[sample[1]][0]
        y_2 = points[sample[1]][1]
        if x_1 == 0 and y_1 == 0:
            continue
        if x_2 == 0 and y_2 == 0:
            continue
        if x_1==x_2:
            a = -1
        else:
            a = (y_2 -y_1) / (x_2-x_1)
        b = y_1 -a * x_1

        inliner = 0

        for i in range(len(points)):
            y_a = a * points[i][0] + b
            sigma = y_a - points[i][1]
            if abs(sigma) < bias:
                ban_list.append(points[i])
                inliner += 1

        if inliner > inner_total:
                # if len(points) == 0 or (math.log(1 - pow(inliner / (len(points)),2))) == 0:
                #     continue
                # max_try = math.log(1-error_rate) / math.log(1 - pow(inliner/len(points),2))
                # inner_total = inliner
                best_point1 = points[sample[0]]
                best_point2 = points[sample[1]]

        # for i in range(len(ban_list)):
        #     if len(ban_list) > 1000:
        #         s = set(points)
        #         if ban_list[i] not in s:
        #             continue
        #         points.remove(ban_list[i])
        #         ban_list = []
        
    
    



map_point_cloud = [(0.0,0.0)]*2
pcl_list = []

def PT_callback(data):
    global pcl_list
    pcl_list = [i for i in data[1:-1].split(",")]
    for i in len(pcl_list):
        n = 10^5
        pcl_list[i][0] = math.floor(pcl_list[i][0]*n)/n
        pcl_list[i][1] = math.floor(pcl_list[i][1]*n)/n






def ransac():
    global pcl_list
    rospy.init_node('runtest')
    rospy.Subscriber('/test', String, PT_callback,queue_size=361)
    wall_pub = rospy.Publisher('/wall_test', Marker,queue_size=10)


    wall = Marker()
    wall.header.frame_id = '/odom'
    wall.type = Marker.LINE_LIST
    wall.scale.x = 0.05
    wall.scale.y = 0.05
    wall.color.b = 1.0
    wall.color.g = 1.0
    wall.color.a = 10.0



    while not rospy.is_shutdown():

        find_wall(len(pcl_list),pcl_list)
        #
        def bk2Point(data):
            p = Point()
            p.x = float(data[0])
            p.y = float(data[1])
            return p
        print (best_point1, best_point2)
        wall.points.append(bk2Point(best_point1))
        wall.points.append(bk2Point(best_point2))

        # for i in range(len(ban_list)):
        #     if len(ban_list) > 1:
        #         s = set(map_point_cloud)
        #         if ban_list[i] not in s:
        #             continue
        #         map_point_cloud.remove(ban_list[i])
        #         ban_list = []

        wall_pub.publish(wall)

        
    rospy.spin()



if __name__== '__main__':
    try:
        ransac()

    except rospy.ROSInterruptException:
        pass

#TPEMIST/2021OCT