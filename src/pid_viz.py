#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray,Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

if __name__ == "__main__":
    rospy.init_node("viz_node")
    
    ref_pub = rospy.Publisher("car_1/ref",Marker,queue_size=200)
    rate = rospy.Rate(10)
    x_array = list(range(0,201))
    x_array.insert(16,15)
    x_array.insert(32,30)
    x_array.insert(48,45)
    x_array.insert(64,60)
    x_array = np.array(x_array)
    y_array = np.zeros(205)
    refs = [0,2,5,10,15]
    y_array[0:16] = 0
    y_array[16:32] = 2
    y_array[32:48] = 5
    y_array[48:64] = 10
    y_array[64:205] = 15
    xy = np.vstack((x_array, y_array)).T
    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = 0.15, 0.15, 0.15
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = 1,1,0
        marker.pose.orientation.w = 1.0
        for i in range(205):
            marker.id = i
            marker.points.append(Point(int(xy[i][0]) , int(xy[i][1]), 0))
            marker.colors.append(ColorRGBA(1 ,1, 0,1.0))
        # print(marker)
        ref_pub.publish(marker)

        rate.sleep()