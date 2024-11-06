#!/usr/bin/env python3 
#!coding=utf-8 

import rospy 
import numpy as np 
from sensor_msgs.msg import Image 
import cv2 
from cv_bridge import CvBridge, CvBridgeError 

import cmd 
from geometry_msgs.msg import TransformStamped 
import message_filters 
import tf 
import pdb 

#Huazi First Calibration 
R_proj = np.array([[606.5848197645856, 0, 315.4543690396861],[0, 606.5893791142095, 253.6488689005001],[0, 0, 1]]) 
distCoeffs = np.float32([0.132719051681050, -0.281364317368024, 0.001922455351845, -0.001576279968731]) 
R_camera2hand = tf.transformations.quaternion_matrix(np.array([-0.67586792546948649, 0.73697422961731296, -0.0064498875476394161, 0.005470937046227713]))[0:3, 0:3] 
T_camera2hand = np.array([0.04934885656870678, 0.014063642867495733, 0.033125237513860394]).reshape([3, 1]) 

#Huazi Second Calibration 
R_proj = np.array([[606.3749216809975, 0, 316.6981623485034],[0, 606.6693511370747, 253.4722893527974],[0, 0, 1]]) 
distCoeffs = np.float32([0.133279631393024, -0.265966410747563, 0.002317279500949, -0.001036436752524]) 
R_camera2hand = tf.transformations.quaternion_matrix(np.array([-0.66782827937860811, 0.74396670685633948, -0.0043834158275223682, 0.022354284091081349]))[0:3, 0:3] 
T_camera2hand = np.array([0.048323995086779217, -0.020206097551988714, 0.046208705211213084]).reshape([3, 1]) 

M_camera2hand = np.hstack([R_camera2hand, T_camera2hand]) 
M_camera2hand = np.vstack([M_camera2hand, np.array([0, 0, 0, 1])]) 

M_hand2camera = np.linalg.inv(M_camera2hand) 
R_hand2camera = M_hand2camera[0:3, 0:3] 
T_hand2camera = M_hand2camera[0:3, 3].reshape([3, 1]) 
print(T_hand2camera) 

def callback(camera_data, hand_data, target_data): 
    global count, bridge 

    if count <= 100000: 
        #print(count) 
        cv_img = bridge.imgmsg_to_cv2(camera_data, "bgr8") 
        timestr = "%.6f" %  camera_data.header.stamp.to_sec() 
        image_name = str(count) + ".jpg" #图像命名：时间戳.jpg 

        #Coordinate Transformation 
        right_img = cv_img 
        right_img = cv2.undistort(right_img, R_proj, distCoeffs) 
        ori_image = right_img 
        #Hand R T 
        quat_hand = np.array([hand_data.transform.rotation.x, hand_data.transform.rotation.y, hand_data.transform.rotation.z, hand_data.transform.rotation.w]) 
        trans_hand = np.array([hand_data.transform.translation.x, hand_data.transform.translation.y, hand_data.transform.translation.z]) 
        R_hand2vicon = tf.transformations.quaternion_matrix(quat_hand)[0:3, 0:3] 
        T_hand2vicon = trans_hand.reshape([3, 1]) 

        #print(quaternion_hand) 
        #print(R_hand2vicon, T_hand2vicon) 
        M_hand2vicon = np.hstack([R_hand2vicon, T_hand2vicon.reshape([3, 1])]) 
        M_hand2vicon = np.vstack([M_hand2vicon, np.array([0, 0, 0, 1])]) 

        M_vicon2hand = np.linalg.inv(M_hand2vicon) 

        #Target R T 
        quat_target = np.array([target_data.transform.rotation.x, target_data.transform.rotation.y, target_data.transform.rotation.z, target_data.transform.rotation.w]) 
        trans_target = np.array([target_data.transform.translation.x, target_data.transform.translation.y, target_data.transform.translation.z]) 
        R_target2vicon = tf.transformations.quaternion_matrix(quat_target)[0:3, 0:3] 
        T_target2vicon = trans_target.reshape([3, 1]) 

        M_target2vicon = np.hstack([R_target2vicon, T_target2vicon.reshape([3, 1])]) 
        M_target2vicon = np.vstack([M_target2vicon, np.array([0, 0, 0, 1])]) 

        M_vicon2target = np.linalg.inv(M_target2vicon) 

        M_target2camera = M_hand2camera.dot(M_vicon2hand).dot(M_target2vicon) 
        R_target2camera = M_target2camera[0:3, 0:3] 
        T_target2camera = M_target2camera[0:3, 3].reshape([3,1]) 
             
        marker_pos = M_target2camera[0:3, 3].reshape([3,1]) 
        print(marker_pos) 
        #print(R_car2vicon) 
        #print(p_o2c) 
        #print(M_hand2camera[0:3, 0:3], M_hand2camera[0:3, 3]) 
        m_proj = R_proj.dot(marker_pos) 
        #print(m_proj) 
        m_proj = m_proj / m_proj[2] 

        #print(m_proj) 
        p_car_x = np.array([0.1, 0, 0]).reshape([3, 1]) 
        p_car_y = np.array([0, 0.1, 0]).reshape([3, 1]) 
        p_car_z = np.array([0, 0, 0.1]).reshape([3, 1]) 
        p_car_camera_x = R_target2camera.dot(p_car_x) + T_target2camera 
        p_car_camera_y = R_target2camera.dot(p_car_y) + T_target2camera 
        p_car_camera_z = R_target2camera.dot(p_car_z) + T_target2camera 

        m_proj_car_x = R_proj.dot(p_car_camera_x) 
        m_proj_car_y = R_proj.dot(p_car_camera_y) 
        m_proj_car_z = R_proj.dot(p_car_camera_z) 

        m_proj_car_x = m_proj_car_x / m_proj_car_x[2] 
        m_proj_car_y = m_proj_car_y / m_proj_car_y[2] 
        m_proj_car_z = m_proj_car_z / m_proj_car_z[2] 

        right_img = cv2.circle(right_img, (int(m_proj[0]), int(m_proj[1])), 1, (255, 0, 0), 2) 
        right_img = cv2.line(right_img, (int(m_proj_car_x[0]), int(m_proj_car_x[1])),(int(m_proj[0]), int(m_proj[1])),(255, 0, 0),3) 
        right_img = cv2.line(right_img, (int(m_proj_car_y[0]), int(m_proj_car_y[1])),(int(m_proj[0]), int(m_proj[1])),(0, 255, 0),3) 
        right_img = cv2.line(right_img, (int(m_proj_car_z[0]), int(m_proj_car_z[1])),(int(m_proj[0]), int(m_proj[1])),(0, 0, 255),3) 

        right_img = cv2.circle(right_img, (int(m_proj[0]), int(m_proj[1])), 1, (255, 0, 0), 2) 
        count += 1 
        cv2.imshow('test.jpg', right_img) 
        cv2.waitKey(1) 
    else: 
        pass 

def displayWebcam(): 
    rospy.init_node('webcam_display', anonymous=True) 

    # make a video_object and init the video object 
    global count, bridge 
    count = 0 
    bridge = CvBridge() 

    camera_img = message_filters.Subscriber('/usb_cam/image_raw', Image, queue_size=1) 
    #camera_img = message_filters.Subscriber('/camera/realsense', Image, queue_size=1) 
    hand = message_filters.Subscriber('/vicon/realsense/realsense', TransformStamped, queue_size=1) 
    target = message_filters.Subscriber('/vicon/TargetTag/TargetTag', TransformStamped, queue_size=1) 

    ts = message_filters.ApproximateTimeSynchronizer([camera_img, hand, target], 10, 0.1, allow_headerless = True) 
    ts.registerCallback(callback) 

    rospy.spin() 

if __name__ == '__main__': 
    while not rospy.is_shutdown(): 
        displayWebcam() 