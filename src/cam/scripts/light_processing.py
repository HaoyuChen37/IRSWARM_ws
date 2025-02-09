import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped
import rospy
import tf


car_ID = 1


# 外参矩阵
cam_homogeneous_matrices = [
    np.array([
        [0.72119598,  0.10725118,  0.68437822,  0.04008849],
        [-0.69155041,  0.05381167,  0.720321,    0.05381581],
        [0.04042774, -0.99277464,  0.11297836, -0.10454239],
        [0.,          0.,          0.,          1.        ]
    ]),
    np.array([
        [-0.72011037, -0.03127949,  0.69315413,  0.06132023],
        [-0.69241984, -0.03192476, -0.72078817, -0.03504754],
        [0.04467466, -0.9990007,   0.00133081, -0.01705665],
        [0.,          0.,          0.,          1.        ]
    ]),
    np.array([
        [-0.70108422, -0.01602186, -0.71289847, -0.0414114],
        [0.71296302,  0.00224053, -0.70119805, -0.06623567],
        [0.01283176, -0.99986913,  0.00985219, -0.02437643],
        [0.,          0.,          0.,          1.        ]
    ]),
    np.array([
        [0.72243153,  0.02749143, -0.69089572, -0.07127752],
        [0.68931443,  0.04969853,  0.72275561,  0.03802169],
        [0.05420608, -0.99838584,  0.01695355, -0.03157856],
        [0.,          0.,          0.,          1.        ]
    ]),
    np.array([
        [0.6891949,   0.00746342,  0.72453757,  0.04061265],
        [-0.7239904, -0.03310119,  0.68901539,  0.05888463],
        [0.02912546, -0.99942414, -0.01740973, -0.00541817],
        [0.,          0.,          0.,          1.        ]
    ]),
    np.array([
        [-0.73037417, -0.02360289,  0.68263934,  0.06172566],
        [-0.68184366, -0.03410973, -0.73070223, -0.03933543],
        [0.04053133, -0.99913934,  0.00881939, -0.01045818],
        [0.,          0.,          0.,          1.        ]
    ]),
    np.array([
        [-0.68541455, -0.02824068, -0.72760522, -0.04325892],
        [0.72786528,  0.00151883, -0.68571848, -0.06084607],
        [0.02047026, -0.9996,      0.01951439, -0.00739562],
        [0.,          0.,          0.,          1.        ]
    ]),
    np.array([
        [0.72994282,  0.00838439, -0.68345679, -0.0618053],
        [0.68330577,  0.015384,    0.72997025,  0.03678699],
        [0.01663465, -0.99984651,  0.00550034, -0.01089537],
        [0.,          0.,          0.,          1.        ]
    ])
]


def get_homogenious(quaternion, position): 
        # 将四元数转换为欧拉角，返回matrix
        R = tf.transformations.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        t = np.array([position.x, position.y, position.z])
        T = np.zeros([4, 4])
        T[0:3, 0:3] = R[0:3, 0:3]
        T[0,3] = t[0]
        T[1,3] = t[1]
        T[2,3] = t[2]
        T[3, 3] = 1
        return T


class LightLocalizer():
    def __init__(self):
        # 初始化5个小车的齐次坐标变换矩阵
        self.T_vicon2car1 = np.eye(4)  # Vicon 到 car1 的变换矩阵
        self.T_vicon2car2 = np.eye(4)  # Vicon 到 car2 的变换矩阵
        self.T_vicon2car3 = np.eye(4)  # Vicon 到 car3 的变换矩阵
        self.T_vicon2car4 = np.eye(4)  # Vicon 到 car4 的变换矩阵
        self.T_vicon2car5 = np.eye(4)  # Vicon 到 car5 的变换矩阵
        # 相机参数配置
        self.camera_matrix = np.array([ [436.04379372 ,  0.    ,     322.2056478 ],
                                        [  0.    ,     408.81252158 , 231.98256365],
                                        [  0.    ,       0.         ,  1.        ]])
        self.dist_coeffs = np.array( [[-0.09917725 , 0.1034774  , 0.00054878,  0.0001342 , -0.01694831]])
        # 多相机外参矩阵字典：{ (小车ID, 相机ID): 外参矩阵 }
        self.cam_extrinsics = {
            (1, 0): cam_homogeneous_matrices[0],
            (1, 1): cam_homogeneous_matrices[1],
            (1, 2): cam_homogeneous_matrices[2],
            (1, 3): cam_homogeneous_matrices[3],
            # 添加其他小车和相机组合...
        }
        self.car1_sub = rospy.Subscriber("/vicon/IRSWARM1/IRSWARM1", TransformStamped, self.car1_callback)
        self.car2_sub = rospy.Subscriber("/vicon/IRSWARM2/IRSWARM2", TransformStamped, self.car2_callback)
        self.car3_sub = rospy.Subscriber("/vicon/IRSWARM3/IRSWARM3", TransformStamped, self.car3_callback)
        self.car4_sub = rospy.Subscriber("/vicon/IRSWARM4/IRSWARM4", TransformStamped, self.car4_callback)
        self.car5_sub = rospy.Subscriber("/vicon/IRSWARM5/IRSWARM5", TransformStamped, self.car5_callback)

    def car1_callback(self, msg):
        # vicon消息
        self.T_vicon2car1 = get_homogenious(msg.pose.orientation, msg.pose.position)
        
    def car2_callback(self, msg):
        # vicon消息
        self.T_vicon2car2 = get_homogenious(msg.pose.orientation, msg.pose.position)
    
    def car3_callback(self, msg):
        # vicon消息
        self.T_vicon2car3 = get_homogenious(msg.pose.orientation, msg.pose.position)
        
    def car4_callback(self, msg):
        # vicon消息
        self.T_vicon2car4 = get_homogenious(msg.pose.orientation, msg.pose.position)
    
    def car5_callback(self, msg):
        # vicon消息
        self.T_vicon2car5 = get_homogenious(msg.pose.orientation, msg.pose.position)
        
    def project_light_to_image(self, car_id, cam_id):
            """核心投影函数：从VICON坐标系到图像平面"""
            try:
                # 1. 获取当前小车的位姿
                T_vicon_to_car = {
                    1: self.vicon.T_vicon2car1,
                    2: self.vicon.T_vicon2car2,
                    3: self.vicon.T_vicon2car3,
                    4: self.vicon.T_vicon2car4,
                    5: self.vicon.T_vicon2car5
                }[car_id]
                
                # 2. 获取相机外参矩阵（小车到相机）
                T_car_to_cam = self.cam_extrinsics[(car_id, cam_id)]
                
                # 3. 组合变换矩阵：VICON -> 小车 -> 相机
                T_vicon_to_cam = T_car_to_cam @ np.linalg.inv(T_vicon_to_car)
                
                # 4. 转换光源坐标到相机坐标系
                light_pos_vicon = T_vicon_to_car @ self.light_pos_car
                light_pos_cam = T_vicon_to_cam @ light_pos_vicon
                
                # 5. 考虑畸变的投影
                cam_coords = light_pos_cam[:3].reshape(1, 3)
                proj_points, _ = cv2.projectPoints(
                    cam_coords, 
                    np.zeros(3),  # 假设无旋转平移
                    np.zeros(3),
                    self.camera_matrix,
                    self.dist_coeffs
                )
                u, v = proj_points[0][0].astype(int)
                
                return (u, v), light_pos_vicon[:3]
            except KeyError:
                return None, None

def process_frame(frame):
    """
    对输入的图像帧进行处理，识别图像中的亮点并计算其像素值总和。

    参数:
        frame (numpy.ndarray): 输入的图像帧（单通道或三通道）。

    返回:
        list: 包含每个亮点的像素值总和的列表。
    """
    # 计算图像的平均像素值
    frame_ave_value = cv2.mean(frame)[0]

    # 设置阈值为图像平均像素值的一定倍数
    threshold = 2 * frame_ave_value
    _, mask1 = cv2.threshold(frame, threshold, 255, cv2.THRESH_BINARY)

    # 确保 mask 是 8 位单通道图像
    if len(mask1.shape) == 3:  # 如果 mask 是三通道，转换为单通道
        mask1 = cv2.cvtColor(mask1, cv2.COLOR_BGR2GRAY)

    # 查找蒙版中的轮廓（即光源区域）
    contours, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centers = []
    pixel_sums = []

    # 如果没有轮廓，直接返回像素值总和为 0
    if not contours:
        pass

    # 遍历每个轮廓
    for contour in contours:
        contour_area = cv2.contourArea(contour)  # 计算轮廓的面积

        # 计算每个亮点的矩（moments），用于计算中心
        moments = cv2.moments(contour)
        center_x = 0
        center_y = 0
        if moments["m00"] != 0:  # 避免除以零（像素个数不为0）
            center_x = int(moments["m10"] / moments["m00"])
            center_y = int(moments["m01"] / moments["m00"])

        if center_y > 150:
            if contour_area >= 80:  # 不太可能出现
                pixel_sum = 1  # 没有光源但有vicon
                # pixel_sums.append(pixel_sum)
            elif 2 < contour_area < 80.0:  # 忽略过小的噪声且只处理面积小于80的轮廓
                # 计算每个亮点的像素值总和
                mask2 = np.zeros_like(frame, dtype=np.uint8)
                if len(mask2.shape) == 3:  # 如果 mask 是三通道，转换为单通道
                    mask2 = cv2.cvtColor(mask2, cv2.COLOR_BGR2GRAY)
                cv2.drawContours(mask2, [contour], -1, (255, 255, 255), thickness=cv2.FILLED)
                masked_image = cv2.bitwise_and(frame, frame, mask=mask2)
                pixel_sum = cv2.sumElems(masked_image)[0]
                pixel_sums.append(pixel_sum)
                centers.append((center_x, center_y))
            else:  # 位置较低的噪声（窗户）
                pixel_sum = 2
                # pixel_sums.append(pixel_sum)
        else:  # vicon或位置较高的噪声（窗户）
            pixel_sum = 1  # 没有光源但有vicon
            # pixel_sums.append(pixel_sum)

    # # 如果 pixel_sums 为空，则赋值为 [0]
    # if not pixel_sums:
    #     pixel_sums = [0]

    return centers, pixel_sums