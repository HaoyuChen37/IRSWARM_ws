import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped
import rospy
import tf


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
        # 添加ROI半径参数
        self.roi_radius = 30  # 像素单位
        # 添加小车坐标接收端
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

    def create_roi_mask(self, projected_pt, img_shape):
        """根据投影点创建圆形ROI掩膜"""
        mask = np.zeros(img_shape[:2], dtype=np.uint8)
        if projected_pt is not None:
            u, v = projected_pt
            # 检查坐标是否在图像范围内
            if 0 <= u < img_shape[1] and 0 <= v < img_shape[0]:
                cv2.circle(mask, (u, v), self.roi_radius, 255, -1)
        return mask
    
    def project_light_to_image(self, self_car_id, self_cam_id, target_car_id, target_cam_id):
            """核心投影函数：从VICON坐标系到图像平面"""
            try:
                # 1. 获取当前小车的位姿
                T_vicon_to_car = {
                    1: self.vicon.T_vicon2car1,
                    2: self.vicon.T_vicon2car2,
                    3: self.vicon.T_vicon2car3,
                    4: self.vicon.T_vicon2car4,
                    5: self.vicon.T_vicon2car5
                }[self_car_id]
                
                # 2. 获取相机外参矩阵（小车到相机）
                T_car_to_cam = self.cam_extrinsics[(self_car_id, self_cam_id)]
                
                # 3. 组合变换矩阵：VICON -> 小车 -> 相机
                T_vicon_to_cam = T_car_to_cam @ np.linalg.inv(T_vicon_to_car)
                
                #  4. 转换光源坐标到相机坐标系
                # 假设小车的位置坐标就是光源坐标
                light_pos_vicon = np.array([0, 0, 0, 1])  # 小车坐标系下的光源位置
                light_pos_vicon = T_vicon_to_car @ light_pos_vicon  # 转换到VICON坐标系
                light_pos_cam = T_vicon_to_cam @ light_pos_vicon  # 转换到相机坐标系
                
                # 5. 考虑畸变的投影
                cam_coords = light_pos_cam[:3].reshape(1, 3)
                proj_points, _ = cv2.projectPoints(
                    cam_coords, 
                    cv2.Rodrigues(R),  # 假设无旋转平移
                    np.zeros(3),
                    self.camera_matrix,
                    self.dist_coeffs
                )
                u, v = proj_points[0][0].astype(int)
                
                return (u, v)
            except KeyError:
                return None

    def process_roi(self, frame, roi_mask):
            """在ROI区域内处理图像并返回光源信息"""
            # 应用ROI掩膜
            masked_frame = cv2.bitwise_and(frame, frame, mask=roi_mask)
            
            # 计算图像的平均像素值（仅在ROI区域内）
            roi_mean = cv2.mean(frame, mask=roi_mask)[0]
            
            # 动态调整阈值
            threshold = 2 * roi_mean if roi_mean > 10 else 50
            _, binary = cv2.threshold(masked_frame, threshold, 255, cv2.THRESH_BINARY)
            
            # 查找轮廓
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            centers = []
            pixel_sums = []
            
            for contour in contours:
                # 轮廓面积过滤
                area = cv2.contourArea(contour)
                if 4 < area < 80:
                    # 计算质心
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # 计算像素值总和
                        mask = np.zeros_like(frame)
                        cv2.drawContours(mask, [contour], -1, 255, -1)
                        pixel_sum = cv2.sumElems(frame, mask=mask)[0]
                        
                        centers.append((cx, cy))
                        pixel_sums.append(pixel_sum)
            
            return centers, pixel_sums
    
    def process_frame(self, frame, car_id=1, cam_id=0):
        """完整的定位流程"""
        # 获取投影点
        projected_pt = self.project_light_to_image(car_id, cam_id)

        # 创建ROI掩膜
        roi_mask = self.create_roi_mask(projected_pt, frame.shape)

        # 处理ROI区域
        centers, pixel_sums = self.process_roi(frame, roi_mask)
        return centers, pixel_sums