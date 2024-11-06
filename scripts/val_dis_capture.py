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

def get_matrix_from_quaternion(quaternion): 
        # 将四元数转换为欧拉角，返回matrix
        matrix = tf.transformations.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return matrix  # 返回matrix

class Camera(object):
    def __init__(self, width=1280, height=720, fps=30):
        rospy.init_node('mindvision_camera_node', anonymous=True)
        # 订阅vicon信息
        self.vicon_sub = rospy.Subscriber("/vrpn_client_node/mindvision/pose", PoseStamped, self.vicon_callback)
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
        cv2.imshow("Press q to end", frame)

        return frame

    def publish_frame(self, frame):
        ros_image = self.bridge.cv2_to_imgmsg(frame, "mono8")
        ros_image.header.stamp = rospy.Time.now()
        self.image_pub.publish(ros_image)

    
    def release(self):
        # 关闭相机
        mvsdk.CameraUnInit(self.hCamera)

	    # 释放帧缓存
        mvsdk.CameraAlignFree(self.pFrameBuffer)
    
    def vicon_callback(self, msg):
        # vicon消息
        self.pose = msg
        # 例如，打印vicon的位置信息
        print("Received vicon position:", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)



if __name__ == '__main__':
    cam = Camera()
    # 修改为Linux风格的路径
    image_dir = r"/home/chenhaoyu/IROS_ws/val_dis/images"
    pose_dir = r"/home/chenhaoyu/IROS_ws/val_dis/poses"
    if not os.path.exists(image_dir):
        # 在Linux中创建目录
        os.makedirs(image_dir)
    
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

            # 获取当前时间并格式化为字符串
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            # 构建Linux风格的文件名和路径
            image_filename = os.path.join(image_dir, f"{timestamp}.png")

            if cv2.waitKey(1) & 0xFF in [ord('q'), 27]:
                
                # 在保存图像之前
                print(f"Saving image to {image_filename}")

                # 保存图像
                cv2.imwrite(image_filename, frame)
                cam.publish_frame(frame)  # Publish the image as a ROS topic

                pose = cam.pose
                print(pose)

                # 将四元数角转换为旋转矩阵
                quaternion = pose.pose.orientation
                matrix = get_matrix_from_quaternion(quaternion)

                position = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])

                # 检查 pose 目录是否存在，如果不存在则创建
                if not os.path.exists(pose_dir):
                    os.makedirs(pose_dir)

                # 将vicon消息写入mat文件
                pose_filename = os.path.join(pose_dir, f"{timestamp}.mat")
                savemat(pose_filename, {'cam_rotation' : matrix, "cam_pos" : position})   

                break
                
            r.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        cam.release()
        cv2.destroyAllWindows()
