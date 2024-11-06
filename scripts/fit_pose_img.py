from scipy.io import savemat
import scipy.io as scio
import os
import glob
import numpy as np
import cv2

pose_path = r'/home/chenhaoyu/inside_exp_ws/EyeInHand/poses/'
img_path = r'/home/chenhaoyu/inside_exp_ws/EyeInHand/images/'
save_path = r'/home/chenhaoyu/inside_exp_ws/EyeInHand/output/'
if not os.path.exists(save_path):
    os.makedirs(save_path)
image_files = glob.glob(os.path.join(img_path, '*.png'))

K_c = np.array([[433.84310735  , 0.           , 316.35195148],
                [  0.          , 407.29303267 , 227.97832544],
                [  0.          , 0.           ,    1.       ]])
distCoeffs = np.array([[-0.10002762 , 0.10331554 , -0.00067012 , -0.0003735 , -0.01315765]])



for image_file in image_files:
    # print(image_file)
    if image_file.split('/')[-1][-4:] == '.png':
        img = cv2.imread(image_file, cv2.IMREAD_UNCHANGED)
        right_img = img
        t = image_file.split('/')[-1][:-4]

        mat_path = pose_path + t + '.mat'
        if os.path.exists(mat_path):
            # read matrix
            matdata = scio.loadmat(mat_path)
            T_vicon_tool = matdata['T_vicon_tool'].reshape(4,4)
            T_vicon_tool = np.array(T_vicon_tool)
            T_tool_vicon = matdata['T_tool_vicon'].reshape(4,4)
            T_tool_vicon = np.array(T_tool_vicon)
            T_camera_tool = matdata['T_camera_tool'].reshape(4,4)
            T_camera_tool = np.array(T_camera_tool)
            T_tool_camera = matdata['T_tool_camera'].reshape(4,4)
            T_tool_camera = np.array(T_tool_camera)
            T_vicon_car = matdata['T_vicon_car'].reshape(4,4)
            T_vicon_car = np.array(T_vicon_car)
            T_car_vicon = matdata['T_car_vicon'].reshape(4,4)
            T_car_vicon = np.array(T_car_vicon)

            # distCoeffs = matdata[distCoeffs]
            # K_c = matdata[K_c]
            
            
            T_camera_car = T_camera_tool.dot(T_tool_vicon).dot(T_vicon_car)
            # print(T_vicon_car)
            car_pos = T_camera_car[0:3, 3].reshape([3,1]) 

            m_proj = K_c.dot(car_pos) 
            # print(m_proj) 
            m_proj = m_proj / m_proj[2] 
            print(m_proj)
            #print(m_proj) 
            p_car_x = np.array([0.1, 0, 0, 1]).reshape([4, 1]) 
            p_car_y = np.array([0, 0.1, 0, 1]).reshape([4, 1]) 
            p_car_z = np.array([0, 0, 0.1, 1]).reshape([4, 1]) 
            p_car_camera_x = T_camera_car.dot(p_car_x)[0:3].reshape([3, 1]) 
            p_car_camera_y = T_camera_car.dot(p_car_y)[0:3].reshape([3, 1]) 
            p_car_camera_z = T_camera_car.dot(p_car_z)[0:3].reshape([3, 1]) 

            m_proj_car_x = K_c.dot(p_car_camera_x) 
            m_proj_car_y = K_c.dot(p_car_camera_y) 
            m_proj_car_z = K_c.dot(p_car_camera_z) 

            m_proj_car_x = m_proj_car_x / m_proj_car_x[2] 
            m_proj_car_y = m_proj_car_y / m_proj_car_y[2] 
            m_proj_car_z = m_proj_car_z / m_proj_car_z[2] 

            right_img = cv2.circle(right_img, (int(m_proj[0]), int(m_proj[1])), 1, (255, 0, 0), 2) 
            right_img = cv2.line(right_img, (int(m_proj_car_x[0]), int(m_proj_car_x[1])),(int(m_proj[0]), int(m_proj[1])),(255, 0, 0),3) 
            right_img = cv2.line(right_img, (int(m_proj_car_y[0]), int(m_proj_car_y[1])),(int(m_proj[0]), int(m_proj[1])),(0, 255, 0),3) 
            right_img = cv2.line(right_img, (int(m_proj_car_z[0]), int(m_proj_car_z[1])),(int(m_proj[0]), int(m_proj[1])),(0, 0, 255),3) 

            right_img = cv2.circle(right_img, (int(m_proj[0]), int(m_proj[1])), 1, (255, 0, 0), 2) 

            cv2.imwrite(save_path + f'{t}.png', right_img) 
            cv2.waitKey(1) 
        else:
            print(mat_path + "dosen't exist.")
