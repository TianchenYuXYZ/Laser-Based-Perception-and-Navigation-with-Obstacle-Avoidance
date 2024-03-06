#!/usr/bin/env python
import roslib
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np
import geometry_msgs.msg
import turtlesim.srv
import tf
import turtlesim.msg
from visualization_msgs.msg import Marker
import random
flagWall = 0
points = []
ranges = np.zeros((361, 2))
pos = 0
orien = 0
front_wall = False
left_line = False
on_line = False
pointset= []


def LS_callback(data):
    global ranges
    global front_wall, left_line
    range = data.ranges
    range = np.array(range)
    counter = 0
    maxsize = 120

    for i in np.arange(maxsize):
        if range[180-(maxsize/2)+i] < 0.5:
            counter += 1
    if counter > 1:
        front_wall = True
    else:
        front_wall = False

    counter = 0
    maxsize = 80

    for i in np.arange(maxsize):
        if range[i] < 1:
            counter += 1
    if counter > 10:
        left_line = True
    else:
        left_line = False
            

def PS_callback(data):
    global orien
    global pos
    pos = data.pose.pose.position
    orien = data.pose.pose.orientation

def WL_callback(data):
    global pointset
    pointset = data.points

def get_angluar_velocity(goal_angle):
    global front_wall, on_line
    if on_line:
        return min(goal_angle, 1)
    elif front_wall:
        return 1
    else:
        return min(goal_angle, 1)
    
def get_angluar_velocity_line_follow(goal_angle):
    global left_line, front_wall
    if front_wall:
        return 0.8
    if len(pointset)>0 and math.atan((pointset[0].y - pos.y) / (pointset[1].x- pos.x)) - robot_angle == 0:
        return 0
    if len(pointset)<0 and left_line:
        return 0
    else:
        return -1 * 0.4

def check_point_on_line(points):
    global pos
    A = points[0,:]
    B = points[1,:]
    C = np.array([pos.x, pos.y])
    area = abs((A[0] * (B[1] - C[1]) + B[0] * (C[1] - A[1]) + C[0] * (A[1] - B[1])) / 2.0)
    threshold = 0.6
    if (area < threshold):
        return True
    else:
        return False



def bug2():

    
    rospy.Subscriber("/base_scan", LaserScan, LS_callback)
    rospy.Subscriber("/base_pose_ground_truth", Odometry, PS_callback)
    rospy.Subscriber("/ransac_wall",Marker,WL_callback)


    global front_wall, orien, pos, on_line
    pubcmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
    rate = rospy.Rate(10)
    points = np.array([[-8, -2],[4.5, 9.0]])
    distance = math.sqrt((points[1,1] - 0) ** 2 + (points[1,0] - 0) ** 2)
    atGoal = False
    thresholddist = 0.5
    follow_type = "GOAL_SEEK"
    
    while not atGoal:
        if orien > 0:
            global robot_angle
            robot_angle = 2 * np.arcsin(orien.z)
            disttemp = math.sqrt((points[1, 0] - pos.x) ** 2 + (points[1, 1] - pos.y) ** 2)
            goal_angle = math.atan((points[1, 1] - pos.y) / (points[1,0]- pos.x)) - robot_angle
            on_line = check_point_on_line(points)
            twist = Twist()
            if disttemp < thresholddist:
                twist.linear.x = 0 
                twist.angular.z = 0
                break
            else:
                vel = 0.0
                if front_wall:
                    vel = 0.0
                else:
                    vel = 0.6
                twist.linear.x = vel
                if follow_type == "GOAL_SEEK":
                    twist.angular.z =  get_angluar_velocity(goal_angle)
                    if front_wall:
                         follow_type = "LINE_FOLLOW"
                else:
                    twist.angular.z = -1 * get_angluar_velocity_line_follow(goal_angle)
                    if on_line and not front_wall:
                        follow_type = "GOAL_SEEK"
            pubcmd_vel.publish(twist)
            rate.sleep()    
          
if __name__ == '__main__':
    try:
        rospy.init_node('bug2')
        bug2()
    except rospy.ROSInterruptException:
        pass


#TPEMIST/2021OCT