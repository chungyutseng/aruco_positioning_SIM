#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv2.aruco as aruco
import math
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Empty as EmptyMsg

rospy.init_node('center', anonymous=True)
# pub_takeoff = rospy.Publisher('/tello/takeoff', EmptyMsg, queue_size=10)
pub_takeoff = rospy.Publisher('/ardrone/takeoff', EmptyMsg, queue_size=10)
pub_desired_x = rospy.Publisher('/desired_x', Float32, queue_size=10)
pub_desired_y = rospy.Publisher('/desired_y', Float32, queue_size=10)
pub_desired_z = rospy.Publisher('/desired_z', Float32, queue_size=10)
pub_desired_yaw = rospy.Publisher('/desired_yaw', Float32, queue_size=10)
pub_control = rospy.Publisher('/controller_on', Float32, queue_size=10)
rate = rospy.Rate(30)

flag = 0
time_old = time.time() - time.time()
marker_detected_flag = 0.0

def get_marker_detected_flag(data):
    global marker_detected_flag
    marker_detected_flag = data.data

def takeoff():
    global pub_takeoff
    time.sleep(5)
    pub_takeoff.publish()
    time.sleep(1)

def pub_on():
    global pub_desired_x, pub_desired_y, pub_desired_z, pub_desired_yaw, pub_control
    global rate
    global marker_detected_flag
    rospy.Subscriber("/marker_detected", Float32, callback=get_marker_detected_flag)
    while not rospy.is_shutdown():
        pub_desired_x.publish(1.7)
        pub_desired_y.publish(0.0)
        pub_desired_z.publish(1.0)
        pub_desired_yaw.publish(0.0)
        if marker_detected_flag == 1.0:
            pub_control.publish(1.0)
        else:
            pub_control.publish(0.0)
        rate.sleep()

def pub_on_1():
    global pub_desired_x, pub_desired_y, pub_desired_z, pub_desired_yaw, pub_control
    global rate
    global flag, time_old
    global marker_detected_flag
    rospy.Subscriber("/marker_detected", Float32, callback=get_marker_detected_flag)
    time_old = time.time()
    while not rospy.is_shutdown():
        time_now = time.time()
        if (time_now - time_old) > 20:
            if flag == 1:
                flag = 0
            else:
                flag = 1
            time_old = time_now
        if flag == 0:
            pub_desired_x.publish(1.8)
            pub_desired_y.publish(0.0)
            pub_desired_z.publish(0.9)
            pub_desired_yaw.publish(0.0)
            pub_control.publish(1.0)
        if flag == 1:
            pub_desired_x.publish(1.5)
            pub_desired_y.publish(0.5)
            pub_desired_z.publish(0.7)
            pub_desired_yaw.publish(0.0)
            pub_control.publish(1.0)
        rate.sleep()

if __name__ == '__main__':
    try:
        takeoff()
        # pub_desired()
        # time.sleep(5)
        # pub_takeoff = rospy.Publisher('/ardrone/takeoff', EmptyMsg, queue_size=10)
        # pub_takeoff.publish()
        # time.sleep(1)
        pub_on()
        # pub_on_1()
        # pub_control = rospy.Publisher('/controller_on', Float32, queue_size=10)
        # pub_control.publish(1.0)
    except rospy.ROSInterruptException:
        pass