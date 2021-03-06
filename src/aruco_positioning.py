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

marker_size = 10
calib_path = ""
# camera_matrix = np.loadtxt('/home/chungyu/.ros/cameraMatrix_Ardrone.txt', delimiter = ',')
# camera_distortion = np.loadtxt('/home/chungyu/.ros/cameraDistortion_Ardrone.txt', delimiter = ',')
camera_matrix = np.loadtxt('/home/chungyu/.ros/cameraMatrix_RealSense.txt', delimiter = ',')
camera_distortion = np.loadtxt('/home/chungyu/.ros/cameraDistortion_RealSense.txt', delimiter = ',')
R_flip = np.zeros((3, 3), dtype = np.float32)
# R_flip[0, 0] = 1
# R_flip[1, 2] = -1
# R_flip[2, 1] = 1
R_flip[0, 1] = -1
R_flip[1, 2] = -1
R_flip[2, 0] = 1
font = cv2.FONT_HERSHEY_PLAIN

roll_camera = 0.0
pitch_camera = 0.0
yaw_camera = 0.0
x_camera = 0.0
y_camera = 0.0
z_camera = 0.0

cmd_vel_linear_x = 0.0
cmd_vel_linear_y = 0.0
cmd_vel_linear_z = 0.0
cmd_vel_angular_z = 0.0

marker_detected_flag = 0.0

x_offset = 18.2941656835168 * 0.01
y_offset = -0.2538466017278 * 0.01
# z_offset =  (7.3461554016761 - 0.0399999991374) * 0.01
z_offset =  (7.3461554016761 - 3.99999991374) * 0.01

#Transformation matrix from drone image frame to marker frame
# rotation_array_c2m = np.zeros((9,), dtype=np.float32)
# translation_array_c2m = np.zeros((3,), dtype=np.float32)
transformation_array_c2m = np.zeros((16,), dtype=np.float32)

# aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_100)
parameters = aruco.DetectorParameters_create()
# parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
board_ids = np.array([[0]], dtype = np.int32)
# board_corners = [np.array([[0.0, 0.0, 0.1], [0.1, 0.0, 0.1], [0.1, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype = np.float32)] # clockwise, beginning from the top-left corner
# board_corners = [np.array([[0.0, 0.0, 0.2], [0.2, 0.0, 0.2], [0.2, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype = np.float32)] # clockwise, beginning from the top-left corner
# board_corners = [np.array([[0.0, 0.33, 0.33], [0.0, 0.0, 0.33], [0.0, 0.0, 0.0], [0.0, 0.33, 0.0]], dtype = np.float32)] # clockwise, beginning from the bottom-left corner
board_corners = [np.array([[2.9975, 0.165, 1.165], [2.9975, -0.165, 1.165], [2.9975, -0.165, 0.835], [2.9975, 0.165, 0.835]], dtype = np.float32)] # clockwise, beginning from the bottom-left corner
# board_corners = [np.array([[0.0, 0.165, 1.165], [0.0, -0.165, 1.165], [0.0, -0.165, 0.835], [0.0, 0.165, 0.835]], dtype = np.float32)] # clockwise, beginning from the bottom-left corner
board = aruco.Board_create(board_corners, aruco_dict, board_ids)

# board_ids_Q = np.array([[0]], dtype = np.int32)
# board_corners_Q = [np.array([[0.0, 0.33, 0.33], [0.0, 0.0, 0.33], [0.0, 0.0, 0.0], [0.0, 0.33, 0.0]], dtype = np.float32)] # clockwise, beginning from the bottom-left corner
# board_Q = aruco.Board_create(board_corners_Q, aruco_dict, board_ids_Q)

pub_x = rospy.Publisher("/x", Float32, queue_size=10)
pub_y = rospy.Publisher("/y", Float32, queue_size=10)
pub_z = rospy.Publisher("/z", Float32, queue_size=10)
pub_roll = rospy.Publisher("/roll", Float32, queue_size=10)
pub_pitch = rospy.Publisher("/pitch", Float32, queue_size=10)
pub_yaw = rospy.Publisher("/yaw", Float32, queue_size=10)
pub_marker_detected_flag = rospy.Publisher("/marker_detected", Float32, queue_size=10)
pub_transformation_array_positioning = rospy.Publisher('/transformation_array_positioning', numpy_msg(Floats), queue_size=10)
# pub_rot_array_positioning = rospy.Publisher('rot_array_positioning', numpy_msg(Floats),queue_size=10)
# pub_trans_array_positioning = rospy.Publisher('trans_array_positioning', numpy_msg(Floats),queue_size=10)

rospy.init_node("aruco_positioning", anonymous=True)

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert(isRotationMatrix(R))

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

#########################################################
# '''
# phi -> theta -> psi euler angle R_z(phi) * R_y(theta) * R_x(psi) first multiply first rotate
# '''
#########################################################

def get_cmd_vel_linear_x(data):
    global cmd_vel_linear_x
    cmd_vel_linear_x = data.data

def get_cmd_vel_linear_y(data):
    global cmd_vel_linear_y
    cmd_vel_linear_y = data.data

def get_cmd_vel_linear_z(data):
    global cmd_vel_linear_z
    cmd_vel_linear_z = data.data

def get_cmd_vel_angular_z(data):
    global cmd_vel_angular_z
    cmd_vel_angular_z = data.data

def convert_color_image(ros_image):
    global roll_camera, pitch_camera, yaw_camera, x_camera, y_camera, z_camera
    global cmd_vel_linear_x, cmd_vel_linear_y, cmd_vel_linear_z, cmd_vel_angular_z
    global marker_detected_flag
    # global rotation_array_c2m, translation_array_c2m
    global transformation_array_c2m
    bridge = CvBridge()
    try:
        color_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray_image, aruco_dict, parameters = parameters)

        # print(ids)

        if ids is None:
            ids = np.array([[-1], [-1]], dtype=np.float32)

        if (np.any(ids[:] == 0)):
        # if len(corners) > 0:
            marker_detected_flag = 1.0

            retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, camera_matrix, camera_distortion, None, None)
            # print(rvec)
            # print(tvec)

            # retval_Q, rvec_Q, tvec_Q = aruco.estimatePoseBoard(corners, ids, board_Q, camera_matrix, camera_distortion, None, None)

            # aruco.drawAxis(color_image, camera_matrix, camera_distortion, rvec, tvec, 0.1)

            # R_ct_Q = np.matrix(cv2.Rodrigues(rvec_Q)[0])
            # R_tc_Q = R_ct_Q.T            
            # pos_camera_Q = -R_tc_Q * np.matrix(tvec_Q)

            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc = R_ct.T

            # print(R_ct[0,0])
            # print(np.matrix(tvec))

            pos_camera = -R_tc * np.matrix(tvec)

            ########################################################
            # rotation_array_c2m[0] = R_tc[0, 0]
            # rotation_array_c2m[1] = R_tc[0, 1]
            # rotation_array_c2m[2] = R_tc[0, 2]
            # rotation_array_c2m[3] = R_tc[1, 0]
            # rotation_array_c2m[4] = R_tc[1, 1]
            # rotation_array_c2m[5] = R_tc[1, 2]
            # rotation_array_c2m[6] = R_tc[2, 0]
            # rotation_array_c2m[7] = R_tc[2, 1]
            # rotation_array_c2m[8] = R_tc[2, 2]

            # translation_array_c2m[0] = pos_camera[0]
            # translation_array_c2m[1] = pos_camera[1]
            # translation_array_c2m[2] = pos_camera[2]

            transformation_array_c2m[0] = R_tc[0, 0]
            transformation_array_c2m[1] = R_tc[0, 1]
            transformation_array_c2m[2] = R_tc[0, 2]
            transformation_array_c2m[3] = pos_camera[0] - 2.9975

            transformation_array_c2m[4] = R_tc[1, 0]
            transformation_array_c2m[5] = R_tc[1, 1]
            transformation_array_c2m[6] = R_tc[1, 2]
            transformation_array_c2m[7] = pos_camera[1] - (-0.165)

            transformation_array_c2m[8] = R_tc[2, 0]
            transformation_array_c2m[9] = R_tc[2, 1]
            transformation_array_c2m[10] = R_tc[2, 2]
            transformation_array_c2m[11] = pos_camera[2] - 0.835

            transformation_array_c2m[12] = 0.0
            transformation_array_c2m[13] = 0.0
            transformation_array_c2m[14] = 0.0
            transformation_array_c2m[15] = 1.0         
            ########################################################

            roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip * R_tc)
            
            roll_camera = math.degrees(roll_camera)
            pitch_camera = math.degrees(pitch_camera)
            yaw_camera = math.degrees(yaw_camera)

            x_camera = pos_camera[0] 
            y_camera = pos_camera[1] 
            z_camera = pos_camera[2]

            x_camera = x_camera - x_offset
            y_camera = y_camera - y_offset
            z_camera = z_camera - z_offset 

            # str_position = "CAMERA Position x=%4.5f y=%4.5f z=%4.5f"%(pos_camera[0]*100, pos_camera[1]*100, pos_camera[2]*100)
            str_position = "CAMERA Position x=%4.5f y=%4.5f z=%4.5f"%(x_camera*100, y_camera*100, z_camera*100)
            str_attitude = "CAMERA Attitude roll=%4.0f pitch=%4.0f yaw=%4.0f"%(roll_camera, pitch_camera, yaw_camera)
            cmd_vel_drone = "x_vel = %4.3f y_vel = %4.3f z_vel = %4.3f yaw_vel = %4.3f"%(cmd_vel_linear_x, cmd_vel_linear_y, cmd_vel_linear_z, cmd_vel_angular_z)
            cv2.putText(color_image, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(color_image, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(color_image, cmd_vel_drone, (0, 300), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # pub_x.publish(pos_camera[0])
            # pub_y.publish(pos_camera[1])
            # pub_z.publish(pos_camera[2])
            # pub_roll.publish(roll_camera)
            # pub_pitch.publish(pitch_camera)
            # pub_yaw.publish(yaw_camera)

        else:
            marker_detected_flag = 0.0
            transformation_array_c2m = np.zeros((16,), dtype=np.float32)
            # x_camera = -9.99
            # y_camera = -9.99
            # z_camera = -9.99
            # roll_camera = -999
            # pitch_camera = -999
            # yaw_camera = -999
            str_position = "CAMERA Position x= None y= None z= None"
            str_attitude = "CAMERA Attitude roll= None pitch= None yaw= None"
            cmd_vel_drone = "x_vel = None y_vel = None z_vel = None yaw_vel = None"
            cv2.putText(color_image, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(color_image, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(color_image, cmd_vel_drone, (0, 300), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.namedWindow("Color")
        cv2.imshow("Color", color_image)
        cv2.waitKey(10)
        
    except CvBridgeError as e:
        print(e)

rospy.Subscriber("/camera/color/image_raw", Image, callback=convert_color_image, queue_size=10)
# rospy.Subscriber("/ardrone/front/image_raw", Image, callback=convert_color_image, queue_size=10)
rospy.Subscriber("/cmd_vel_linear_x", Float32, callback=get_cmd_vel_linear_x, queue_size=10)
rospy.Subscriber("/cmd_vel_linear_y", Float32, callback=get_cmd_vel_linear_y, queue_size=10)
rospy.Subscriber("/cmd_vel_linear_z", Float32, callback=get_cmd_vel_linear_z, queue_size=10)
rospy.Subscriber("/cmd_vel_angular_z", Float32, callback=get_cmd_vel_angular_z, queue_size=10)

while not rospy.is_shutdown():
    rate = rospy.Rate(30)
    if marker_detected_flag == 1.0:
        pub_x.publish(x_camera)
        pub_y.publish(y_camera)
        pub_z.publish(z_camera)
        pub_roll.publish(roll_camera)
        pub_pitch.publish(pitch_camera)
        pub_yaw.publish(yaw_camera)
    # pub_rot_array_positioning.publish(rotation_array_c2m)
    # pub_trans_array_positioning.publish(translation_array_c2m)
    pub_marker_detected_flag.publish(marker_detected_flag)
    pub_transformation_array_positioning.publish(transformation_array_c2m)
    rate.sleep()
#################################################################################################################################
# def aruco_positioning():
#     rospy.init_node("aruco_positioning", anonymous=True)
#     # rospy.Subscriber("/raw_image", Image, callback=convert_color_image, queue_size=10)
#     rospy.Subscriber("/camera/color/image_raw", Image, callback=convert_color_image, queue_size=10)
#     # rospy.Subscriber("/usb_cam/image_raw", Image, callback=convert_color_image, queue_size=10)
#     # rospy.Subscriber("/tello/camera/image_raw", Image, callback=convert_color_image, queue_size=10)
#     rospy.Subscriber("/cmd_vel_linear_x", Float32, callback=get_cmd_vel_linear_x, queue_size=10)
#     rospy.Subscriber("/cmd_vel_linear_y", Float32, callback=get_cmd_vel_linear_y, queue_size=10)
#     rospy.Subscriber("/cmd_vel_linear_z", Float32, callback=get_cmd_vel_linear_z, queue_size=10)
#     rospy.Subscriber("/cmd_vel_angular_z", Float32, callback=get_cmd_vel_angular_z, queue_size=10)
#     rospy.spin()

# if __name__ == '__main__':
#     try:
#         aruco_positioning()
#         # pub_position()
#     except rospy.ROSInterruptException:
#         pass
#################################################################################################################################


# def pub_position():
#     global pub_x, pub_y, pub_z, pub_roll, pub_pitch, pub_yaw
#     global roll_camera, pitch_camera, yaw_camera, x_camera, y_camera, z_camera
    # rate = rospy.Rate(30)