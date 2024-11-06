import scipy.io as scio
from scipy.io import savemat
import numpy as np
import os
import rosbag
import tf
import pdb
from cv_bridge import CvBridge, CvBridgeError 
import cv2

def get_matrix_from_quaternion(quaternion):
    matrix = tf.transformations.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    return matrix

def get_homogenious(R, t):
    T = np.zeros([4, 4])
    T[0:3, 0:3] = R[0:3, 0:3]
    T[0,3] = t[0]
    T[1,3] = t[1]
    T[2,3] = t[2]
    T[3, 3] = 1
    return T

bag = rosbag.Bag(r'/home/chenhaoyu/2024-08-20-17-00-47.bag')

camera_tool_path = r'/home/chenhaoyu/inside_exp_ws/EyeInHand/EyeInHand.mat'

matdata = scio.loadmat(camera_tool_path)
T_camera_tool = matdata['T_camera_tool'].reshape(4,4)
T_camera_tool = np.array(T_camera_tool)
T_camera_tool[:3, 3] = T_camera_tool[:3, 3] / 1000
T_tool_camera = matdata['T_tool_camera'].reshape(4,4)
T_tool_camera = np.array(T_tool_camera)
T_tool_camera[:3, 3] = T_tool_camera[:3, 3] / 1000
K_c = np.array([[433.84310735  , 0.           , 316.35195148],
                [  0.          , 407.29303267 , 227.97832544],
                [  0.          , 0.           ,    1.       ]])
distCoeffs = np.array([[-0.10002762 , 0.10331554 , -0.00067012 , -0.0003735 , -0.01315765]])

pose_dir = r'/home/chenhaoyu/inside_exp_ws/EyeInHand/poses'
img_dir = r'/home/chenhaoyu/inside_exp_ws/EyeInHand/images'
if not os.path.exists(pose_dir):
    os.makedirs(pose_dir)
if not os.path.exists(img_dir):
    os.makedirs(img_dir)
# 初始化字典来存储时间戳和对应的数据
pose_data = {}

# 遍历bag文件
for topic, msg, t in bag.read_messages():

    t = str(t.to_nsec())[:-7]

    if topic == '/camera/color/image_raw':
        # 检查时间戳是否已经存在于字典中
        cv_img = CvBridge().imgmsg_to_cv2(msg, "mono8")
        print(cv_img.shape)

        image_filename = os.path.join(img_dir, f"{t}.png")
        cv2.imwrite(image_filename, cv_img)

        if t not in pose_data.keys():
            pose_data[t] = {
                'T_vicon_tool': None,
                'T_tool_vicon': None,
                'T_camera_tool': T_camera_tool,  # 假设这是全局的，不需要每次都加载
                'T_tool_camera': T_tool_camera,
                'T_vicon_car': None,
                'T_car_vicon': None,
                'distCoeffs': distCoeffs,
                'K_c': K_c
            }

    if t in pose_data:
        if topic == '/vrpn_client_node/mindvision/pose':
            R_matrix = get_matrix_from_quaternion(msg.pose.orientation)
            position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            T_vicon_tool = get_homogenious(R_matrix, position)
            pose_data[t]['T_vicon_tool'] = T_vicon_tool
            pose_data[t]['T_tool_vicon'] = np.linalg.inv(T_vicon_tool)

        if topic == '/vrpn_client_node/car_light/pose':
            R_matrix = get_matrix_from_quaternion(msg.pose.orientation)
            position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            T_vicon_car = get_homogenious(R_matrix, position)
            pose_data[t]['T_vicon_car'] = T_vicon_car
            pose_data[t]['T_car_vicon'] = np.linalg.inv(T_vicon_car)
# pdb.set_trace()
# 保存每个时间戳的数据到对应的.mat文件
for t, data in pose_data.items():
    output_filename = os.path.join(pose_dir, f"{t}.mat")  # 使用时间戳的纳秒级表示作为文件名
    if not any(value is None for value in data.values()):
        print(output_filename)
        print(data)
        savemat(output_filename, data)