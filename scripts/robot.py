#!/usr/bin/env python
# from re import M
import rospy
# from rospy.core import xmlrpcapi
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Pose
# from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
# import random as rand
# from std_msgs.msg import String
from geometry_msgs.msg import Point,Point32
import math
import tf
from visualization_msgs.msg import Marker
from std_msgs.msg import String
import laser_geometry.laser_geometry as LG

ranges = []
robo_x = 0
robo_y = 0
robo_angular = 0
point_x = 0
point_y = 0
robo_rotation = 0

#store the point to ransac


 
LP = LG.LaserProjection()

def LS_callback(data):
    global ranges,angle_min,angle_increment, pcl2_pub, LP
    ranges = data.ranges
    angle_min = data.angle_min
    angle_increment = data.angle_increment
    pc2_msg = LP.projectLaser(data)
    pcl2_pub.publish(pc2_msg)

    

def PS_callback(data):
    global robo_x, robo_y, robo_rotation
    robo_x = data.pose.pose.position.x
    robo_y = data.pose.pose.position.y
    rota_q = data.pose.pose.orientation
    quaternion = (rota_q.x,rota_q.y,rota_q.z,rota_q.w)
    robo_rotation = tf.transformations.euler_from_quaternion(quaternion)[2]
    
    
class scan2point:
    def __init__(self, robo_x, robo_y, robo_rotation, scan_d, theta ):
        self.point_x = robo_x + (math.cos((robo_rotation + theta))* scan_d)
        self.point_y = robo_y + (math.sin((robo_rotation + theta))* scan_d)
        # strange cos sin

# class data_collection:
#     def __init__(self, point):
        # list_set = set(map_point_cloud)
        # if point not in list_set:
        #     map_point_cloud.append(point)



def robot():
    #Laser
    global ranges,angle_min,angle_increment
    #Position
    global robo_x,robo_y, robo_rotation

    global map_point_cloud, pcl2_pub
    
    
    rospy.init_node('robot')
    rospy.Subscriber('/base_scan', LaserScan, LS_callback,queue_size=1)
    rospy.Subscriber('/base_pose_ground_truth',Odometry, PS_callback)
    pcl_pub = rospy.Publisher('/point_cloud', Marker, queue_size=10)
    data_pub = rospy.Publisher('/points_data', Point,queue_size=1)
    pcl2_pub = rospy.Publisher('/point_cloud2',PointCloud2, queue_size=30)

    

    marker_pcl = Marker()
    marker_pcl.header.frame_id = '/odom'
    marker_pcl.type = Marker.POINTS
    marker_pcl.scale.x = 0.05
    marker_pcl.scale.y = 0.05
    marker_pcl.color.a = 1.0

    # test
    test = PointCloud2()
    test.header.frame_id = '/odom'
    test.height = 1
    test.width = 361

    

    while not rospy.is_shutdown():
        marker_pcl.points = [0] * len(ranges)
        testlist = [0] *len(ranges)
        test.data = []
        for i in range(len(ranges)):
            if ranges[i] < 3:
                theta = angle_min + (angle_increment * i)
                point = scan2point(robo_x, robo_y, robo_rotation, ranges[i],theta)
                
                pcl_point  = Point()
                pcl_point.x = point.point_x
                pcl_point.y = point.point_y
                marker_pcl.points[i] = pcl_point
                marker_pcl.color.g = 1.0
                # test.data[i] = (point.point_x,point.point_y)
                data_pub.publish(pcl_point)

                

            else:
                pcl_point  = Point()
                marker_pcl.points[i] = pcl_point

            # data_pub.publish(marker_pcl.points)
        
        # test_pub.publish(test)
        pcl_pub.publish(marker_pcl)
        

        # ------------------------------------
        safe_detect_system = [i for i in ranges if i <= 0.8]
        
        if any (safe_detect_system):  #///if (len(distance_smaller_than_thres_list)>0):
            rate = rospy.Rate(1)
            rate.sleep()
            # print("!!!")
            # print(ranges[1])

        else:
            rate = rospy.Rate(100)
            rate.sleep()
            # print("Clear to go")
            # print(ranges[1])

    rospy.spin()


if __name__== '__main__':
    try:
        robot()

    except rospy.ROSInterruptException:
        pass

#TPEMIST/2021OCT



