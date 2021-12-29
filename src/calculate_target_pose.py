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
target_roll = 0.0
target_pitch = 0.0
target_yaw = 0.0

transformation_matrix_positioning = np.zeros((4, 4), dtype=np.float32)
transformation_matrix_target = np.zeros((4, 4), dtype=np.float32)
transformation_matrix_positioning_marker_to_world = np.array([[1.0, 0.0, 0.0, 2.9975], [0.0, 1.0, 0.0, -0.165], [0.0, 0.0, 1.0, 0.835], [0.0, 0.0, 0.0, 1.0]], dtype=np.float32)

target_pose = np.zeros((4, 4), dtype=np.float32)
target_rotation_matrix = np.zeros((3, 3), dtype=np.float32)

rospy.init_node("calculate_target_pose", anonymous=True)

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    # print(shouldBeIdentity)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    # print(n)
    return n < 1e-6

#########################################################
# '''
# phi -> theta -> psi euler angle R_z(phi) * R_y(theta) * R_x(psi) first multiply first rotate
# '''
#########################################################
def rotationMatrixToEulerAngles(R):
    # assert(isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])    # psi: X
        y = math.atan2(-R[2, 0], sy)        # theta: Y
        z = math.atan2(R[1, 0], R[0, 0])    # phi: Z 
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

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
    global target_rotation_matrix
    global target_roll, target_pitch, target_yaw
    temp = np.zeros((4, 4), dtype=np.float32)
    temp = transformation_matrix_positioning_marker_to_world.dot(transformation_matrix_positioning)
    target_pose = temp.dot(transformation_matrix_target)
    if (target_pose[3, 3] != 0):
        target_rotation_matrix = target_pose[0:3, 0:3]
        target_roll, target_pitch, target_yaw = rotationMatrixToEulerAngles(target_rotation_matrix)
        target_roll = math.degrees(target_roll)
        target_pitch = math.degrees(target_pitch)
        target_yaw = math.degrees(target_yaw)
    # target_rotation_matrix
    # temp = transformation_matrix_target.dot(transformation_matrix_positioning)
    # target_pose = temp.dot(transformation_matrix_positioning_marker_to_world)

rospy.Subscriber('/transformation_array_positioning', numpy_msg(Floats), callback=get_transformation_array_positioning, queue_size=10)
rospy.Subscriber('/transformation_array_target', numpy_msg(Floats), callback=get_transformation_array_target, queue_size=10)
rospy.Subscriber('/target_detected', Float32, callback=get_target_detected_flag, queue_size=10)

while not rospy.is_shutdown():
    rate = rospy.Rate(30)
    # print(target_pose)
    print("roll = %4.5f pitch = %4.5f yaw = %4.5f"%(target_roll, target_pitch, target_yaw))
    # print(target_rotation_matrix)
    rate.sleep()