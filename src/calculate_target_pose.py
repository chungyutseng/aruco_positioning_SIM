#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv2.aruco as aruco
import math
from std_msgs.msg import Float32
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

transformation_array_positioning = np.zeros((16,), dtype=np.float32)
transformation_array_target = np.zeros((16,), dtype=np.float32)

transformation_matrix_positioning = np.zeros((4, 4), dtype=np.float32)
transformation_matrix_target = np.zeros((4, 4), dtype=np.float32)
transformation_matrix_positioning_marker_to_world = np.array([[1.0, 0.0, 0.0, 2.9975], [0.0, 1.0, 0.0, -0.165], [0.0, 0.0, 1.0, 0.835], [0.0, 0.0, 0.0, 1.0]], dtype=np.float32)

target_pose = np.zeros((4, 4), dtype=np.float32)

rospy.init_node("calculate_target_pose", anonymous=True)

def get_transformation_array_positioning(data):
    global transformation_array_positioning
    global transformation_matrix_positioning
    transformation_array_positioning = data.data
    transformation_matrix_positioning = transformation_array_positioning.reshape(4, 4)

def get_transformation_array_target(data):
    global transformation_array_target
    global transformation_matrix_target
    transformation_array_target = data.data
    transformation_matrix_target = transformation_array_target.reshape(4, 4)

def get_target_detected_flag(data):
    global target_pose
    temp = np.zeros((4, 4), dtype=np.float32)
    temp = transformation_matrix_positioning_marker_to_world.dot(transformation_matrix_positioning)
    target_pose = temp.dot(transformation_matrix_target)
    # temp = transformation_matrix_target.dot(transformation_matrix_positioning)
    # target_pose = temp.dot(transformation_matrix_positioning_marker_to_world)

rospy.Subscriber('/transformation_array_positioning', numpy_msg(Floats), callback=get_transformation_array_positioning, queue_size=10)
rospy.Subscriber('/transformation_array_target', numpy_msg(Floats), callback=get_transformation_array_target, queue_size=10)
rospy.Subscriber('/target_detected', Float32, callback=get_target_detected_flag, queue_size=10)

while not rospy.is_shutdown():
    rate = rospy.Rate(30)
    print(target_pose)
    rate.sleep()