#!/usr/bin/env python
import os
from datetime import datetime
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf
import mvsdk
import platform
from scipy.io import savemat
import pdb

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


cameraMatrix = np.array([[433.84310735  , 0.           , 316.35195148],
                [  0.          , 407.29303267 , 227.97832544],
                [  0.          , 0.           ,    1.       ]])
distCoeffs = np.array([[-0.10002762 , 0.10331554 , -0.00067012 , -0.0003735 , -0.01315765]])

points_dict = {
    0: [
        (-14.5, 14.5, 0),
        (14.5, 14.5, 0),
        (14.5, -14.5, 0),
        (-14.5, -14.5, 0)
    ],
    1: [
        (118.0, 14.5, 0),
        (147.0, 14.5, 0),
        (147.0, -14.5, 0),
        (118.0, -14.5, 0)
    ],
    2: [
        (-14.5, -118.0, 0),
        (14.5, -118.0, 0),
        (14.5, -147.0, 0),
        (-14.5, -147.0, 0)
    ],
    3: [
        (-147.0, 14.5, 0),
        (-118.0, 14.5, 0),
        (-118.0, -14.5, 0),
        (-147.0, -14.5, 0)
    ],
    4: [
        (-14.5, 147.0, 0),
        (14.5, 147.0, 0),
        (14.5, 118.0, 0),
        (-14.5, 118.0, 0)
    ],
    5: [
        (60.75, 175.0, 0),
        (86.75, 175.0, 0),
        (86.75, 149.0, 0),
        (60.75, 149.0, 0)
    ],
    6: [
        (149.0, 86.75, 0),
        (175.0, 86.75, 0),
        (175.0, 60.75, 0),
        (149.0, 60.75, 0)
    ],
    7: [
        (149.0, -60.75, 0),
        (175.0, -60.75, 0),
        (175.0, -86.75, 0),
        (149.0, -86.75, 0)
    ],
    8: [
        (60.75, -149.0, 0),
        (86.75, -149.0, 0),
        (86.75, -175.0, 0),
        (60.75, -175.0, 0)
    ],
    9: [
        (-86.75, -149.0, 0),
        (-60.75, -149.0, 0),
        (-60.75, -175.0, 0),
        (-86.75, -175.0, 0)
    ],
    10: [
        (-175.0, -60.75, 0),
        (-149.0, -60.75, 0),
        (-149.0, -86.75, 0),
        (-175.0, -86.75, 0)
    ],
    11: [
        (-175.0, 86.75, 0),
        (-149.0, 86.75, 0),
        (-149.0, 60.75, 0),
        (-175.0, 60.75, 0)
    ],
    12: [
        (-86.75, 175.0, 0),
        (-60.75, 175.0, 0),
        (-60.75, 149.0, 0),
        (-86.75, 149.0, 0)
    ],
    13: [
        (126.5, 146.5, 0),
        (146.5, 146.5, 0),
        (146.5, 126.5, 0),
        (126.5, 126.5, 0)
    ],
    14: [
        (126.5, -126.5, 0),
        (146.5, -126.5, 0),
        (146.5, -146.5, 0),
        (126.5, -146.5, 0)
    ],
    15: [
        (-146.5, -126.5, 0),
        (-126.5, -126.5, 0),
        (-126.5, -146.5, 0),
        (-146.5, -146.5, 0)
    ],
    16: [
        (-146.5, 146.5, 0),
        (-126.5, 146.5, 0),
        (-126.5, 126.5, 0),
        (-146.5, 126.5, 0)
    ]
}
class Camera(object):
    def __init__(self, width=1280, height=720, fps=30):
        rospy.init_node('mindvision_camera_node', anonymous=True)
        # 订阅vicon信息
        self.cam_sub = rospy.Subscriber("/vrpn_client_node/mindvision/pose", PoseStamped, self.cam_callback)
        self.cali_sub = rospy.Subscriber("/vrpn_client_node/Cali_ruler/pose", PoseStamped, self.cali_callback)
        self.width = width
        self.height = height
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size=10)
        # initialize camera parameters
        self.DevList = []
        self.hCamera = 0
        self.FrameBufferSize = 0
        self.pFrameBuffer = None
        # initialize pose
        self.pose = None
        

    def initialization(self):
    # 枚举相机
        self.DevList = mvsdk.CameraEnumerateDevice()
        nDev = len(self.DevList)
        if nDev < 1:
            print("No camera was found!")
            return
        
        for i, DevInfo in enumerate(self.DevList):
            print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))
        i = 0 if nDev == 1 else int(input("Select camera: "))
        DevInfo = self.DevList[i]
        print(DevInfo)

        # 打开相机
        self.hCamera = 0
        try:
            self.hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
        except mvsdk.CameraException as e:
            print("CameraInit Failed({}): {}".format(e.error_code, e.message) )
            return

        # 获取相机特性描述
        cap = mvsdk.CameraGetCapability(self.hCamera)

        # 判断是黑白相机还是彩色相机
        monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

        # 黑白相机让ISP直接输出MONO数据
        mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        
        # 相机模式切换成连续采集
        mvsdk.CameraSetTriggerMode(self.hCamera, 0)

        # 手动曝光，曝光时间2ms
        mvsdk.CameraSetAeState(self.hCamera, 0)
        mvsdk.CameraSetExposureTime(self.hCamera, 8 * 1000)

        # 让SDK内部取图线程开始工作
        mvsdk.CameraPlay(self.hCamera)

        # 计算RGB buffer所需的大小，这里直接按照相机的最大分辨率来分配
        self.FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)

        # 分配RGB buffer，用来存放ISP输出的图像
        # 备注：从相机传输到PC端的是RAW数据，在PC端通过软件ISP转为RGB数据（如果是黑白相机就不需要转换格式，但是ISP还有其它处理，所以也需要分配这个buffer）
        self.pFrameBuffer = mvsdk.CameraAlignMalloc(self.FrameBufferSize, 16)
    
    def get_frame(self):
        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.hCamera, 200)
        mvsdk.CameraImageProcess(self.hCamera, pRawData, self.pFrameBuffer, FrameHead)
        mvsdk.CameraReleaseImageBuffer(self.hCamera, pRawData)

        # windows下取到的图像数据是上下颠倒的，以BMP格式存放。转换成opencv则需要上下翻转成正的
        # linux下直接输出正的，不需要上下翻转
        if platform.system() == "Windows":
            mvsdk.CameraFlipFrameBuffer(self.pFrameBuffer, FrameHead, 1)
        
        # 此时图片已经存储在pFrameBuffer中，对于彩色相机pFrameBuffer=RGB数据，黑白相机pFrameBuffer=8位灰度数据
        # 把pFrameBuffer转换成opencv的图像格式以进行后续算法处理
        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self.pFrameBuffer)
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3) )

        frame = cv2.resize(frame, (640,480), interpolation = cv2.INTER_LINEAR)
        # cv2.imshow("Press q to end", frame)
        # cv2.waitKey(1)

        return frame

    # def publish_frame(self, frame):
    #     ros_image = self.bridge.cv2_to_imgmsg(frame, "mono8")
    #     ros_image.header.stamp = rospy.Time.now()
    #     self.image_pub.publish(ros_image)

    
    def release(self):
        # 关闭相机
        mvsdk.CameraUnInit(self.hCamera)

	    # 释放帧缓存
        mvsdk.CameraAlignFree(self.pFrameBuffer)
    
    def cam_callback(self, msg):
        # vicon消息
        global M_hand2vicon
        M_hand2vicon = get_homogenious(msg.pose.orientation, msg.pose.position)
        # 例如，打印vicon的位置信息
        # print("Received vicon position:", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def cali_callback(self, msg):
        # vicon消息
        global M_cali2vicon
        M_cali2vicon = get_homogenious(msg.pose.orientation, msg.pose.position)
        # 例如，打印vicon的位置信息
        # print("Received vicon position:", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)




if __name__ == '__main__':
    cam = Camera()
    # 修改为Linux风格的路径
    marker_dir = r"/home/chenhaoyu/mindvision/EyeInHand/marker"
    if not os.path.exists(marker_dir):
        # 在Linux中创建目录
        os.makedirs(marker_dir)
    
    try:
        r = rospy.Rate(30)  # 30hz
        cam.initialization()
        while not rospy.is_shutdown():
            try:
                frame = cam.get_frame()
                if frame is None:
                    continue
            except mvsdk.CameraException as e:
                if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                    print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message) )
            #frame = 255 -frame
            arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
            arucoParams = cv2.aruco.DetectorParameters_create()
            (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
            # print(corners)
            if len(corners) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            point3d = []
            corners2d = []
            for i in range(len(corners)):
                id = ids[i]
                temp_point3d = points_dict[int(id)]
                for j in range(4):
                    point3d.append(temp_point3d[j])
                    corners2d.append(corners[i][0][j])

            point3d = np.array(point3d)
            corners2d = np.array(corners2d)
            #pdb.set_trace()
            (success, rotation_vector, translation_vector) = cv2.solvePnP(point3d, corners2d, cameraMatrix, distCoeffs,                           
            flags=cv2.SOLVEPNP_ITERATIVE)

            R = cv2.Rodrigues(rotation_vector)[0].reshape(3,3)
            T = translation_vector.reshape(3,1)
            T = T / 1000

            M = np.zeros([4, 4])
            M[:3, :3] = R
            M[3, 3] = 1
            M[0, 3] = T[0]
            M[1, 3] = T[1]
            M[2, 3] = T[2]

            m_proj = cameraMatrix.dot(T) 
            # print(m_proj) 
            m_proj = m_proj / m_proj[2] 

            p_car_x = np.array([0.05, 0, 0, 1]).reshape([4, 1]) 
            p_car_y = np.array([0, 0.05, 0, 1]).reshape([4, 1]) 
            p_car_z = np.array([0, 0, 0.05, 1]).reshape([4, 1]) 
            p_car_camera_x = M.dot(p_car_x)[:3].reshape([3, 1])
            p_car_camera_y = M.dot(p_car_y)[:3].reshape([3, 1])
            p_car_camera_z = M.dot(p_car_z)[:3].reshape([3, 1])

            m_proj_car_x = cameraMatrix.dot(p_car_camera_x) 
            m_proj_car_y = cameraMatrix.dot(p_car_camera_y) 
            m_proj_car_z = cameraMatrix.dot(p_car_camera_z) 

            m_proj_car_x = m_proj_car_x / m_proj_car_x[2] 
            m_proj_car_y = m_proj_car_y / m_proj_car_y[2] 
            m_proj_car_z = m_proj_car_z / m_proj_car_z[2] 

            frame = cv2.circle(frame, (int(m_proj[0]), int(m_proj[1])), 1, 255, 2) 
            frame = cv2.line(frame, (int(m_proj_car_x[0]), int(m_proj_car_x[1])),(int(m_proj[0]), int(m_proj[1])),255,3) 
            frame = cv2.line(frame, (int(m_proj_car_y[0]), int(m_proj_car_y[1])),(int(m_proj[0]), int(m_proj[1])),0,3) 
            frame = cv2.line(frame, (int(m_proj_car_z[0]), int(m_proj_car_z[1])),(int(m_proj[0]), int(m_proj[1])),155,3) 

            M_cali2cam = M
            M_cam2cali = np.linalg.inv(M_cali2cam)

            M_vicon2hand = np.linalg.inv(M_hand2vicon)
            
            M_cam2hand = M_vicon2hand.dot(M_cali2vicon).dot(M_cam2cali)
            print(M_cam2hand)
            cv2.imshow("frame", frame)
            cv2.waitKey(1)
                
            r.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        cam.release()
        cv2.destroyAllWindows()
