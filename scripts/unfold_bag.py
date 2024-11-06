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
    matrix = matrix[:3,:3]
    return matrix

def get_homogenious(R, t):
    T = np.zeros([4, 4])
    T[0:3, 0:3] = R[0:3, 0:3]
    T[0,3] = t[0]
    T[1,3] = t[1]
    T[2,3] = t[2]
    T[3, 3] = 1
    return T

bag = rosbag.Bag(r'/home/chenhaoyu/2024-10-11-13-14-06.bag')




pose_dir = r'/home/chenhaoyu/IROS_workspace/data/2024-10-11-13-14-06/poses/'
img_dir = r'/home/chenhaoyu/IROS_workspace/data/2024-10-11-13-14-06/images/'
if not os.path.exists(pose_dir):
    os.makedirs(pose_dir)
if not os.path.exists(img_dir):
    os.makedirs(img_dir)
# 初始化字典来存储时间戳和对应的数据
pose_data = {}
index = 0

# 遍历bag文件
for topic, msg, t in bag.read_messages():

    t = t.to_nsec()

    if topic == '/camera/color/image_raw':
        # 检查时间戳是否已经存在于字典中
        cv_img = CvBridge().imgmsg_to_cv2(msg, "mono8")
        # print(cv_img.shape)

        # 计算图像的平均像素值
        mean_val = cv2.mean(cv_img)[0]

        # 设置阈值为图像平均像素值的2倍
        threshold = 2 * mean_val

        # 应用阈值来创建蒙版
        _, mask = cv2.threshold(cv_img, threshold, 255, cv2.THRESH_BINARY)

        # 应用蒙版到原图，这里使用cv2.bitwise_and函数
        masked_img = cv2.bitwise_and(cv_img, cv_img, mask=mask)

        # 找到蒙版中所有非零像素的坐标
        mask_coords = np.column_stack(np.where(mask > 0))

        if mask_coords.size > 0:
            center_x = int(np.mean(mask_coords[:, 1]))
            center_y = int(np.mean(mask_coords[:, 0]))
            print("蒙版的中心坐标为:", (center_x, center_y))
        else:
            print("蒙版中没有非零像素，无法确定中心。")

        pixel_sum = cv2.sumElems(masked_img)[0]

        image_filename = os.path.join(img_dir, f"{t}.png")
        # cv2.imwrite(image_filename, cv_img)

        if t not in pose_data.keys():
            index += 1
            pose_data[t] = {
                'cam_pose': None,
                'cam_R': None,
                'val': pixel_sum,
                'loc': [center_x, center_y],
                'index': index
            }

print(pose_data)

for t, data in pose_data.items():
    diff = t
    for topic, msg, t_fit in bag.read_messages():
        t_fit = t_fit.to_nsec()
        if abs(t_fit - t) < diff:
            if topic == '/vrpn_client_node/mindvision/pose':
                pose_data[t]['cam_pose'] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
                pose_data[t]['cam_R'] = get_matrix_from_quaternion(msg.pose.orientation)
print(pose_data)            

time = np.zeros((1, len(pose_data)))
cam_pose = np.zeros((3, len(pose_data)))
cam_R = np.zeros((3, 3, len(pose_data)))
loc = np.zeros((2, len(pose_data)))
val = np.zeros((1, len(pose_data)))

# pdb.set_trace()
index = 0
# 保存每个时间戳的数据到对应的.mat文件
for t, data in pose_data.items():
    index += 1
    if not any(value is None for value in data.values()):
        time[0, data['index']] = t
        cam_pose[:, data['index']] = pose_data[t]['cam_pose']
        cam_R[:, :, data['index']] = pose_data[t]['cam_R']
        loc[:, data['index']] = pose_data[t]['loc']
        val[0, data['index']] = pose_data[t]['val']

pose_dir = pose_dir + '2024-10-11-13-14-06.mat'
final_data = {'time' : time, 'cam_pose' : cam_pose, 'cam_R' : cam_R, 'loc' : loc, 'val' : val }
print(final_data)
savemat(pose_dir, final_data)
print('save file successfully')