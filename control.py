#!/usr/bin/env python
import os
from datetime import datetime
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
import cv2
import mvsdk
import platform
from scipy.io import savemat
import tf

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

def get_matrix_from_quaternion(quaternion):
    matrix = tf.transformations.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    matrix = matrix[:3,:3]
    return matrix

def record_distance(pose1, pose2):
    dis = ((pose1[0]-pose2[0])**2 + (pose1[1]-pose2[1])**2 + (pose1[2]-pose2[2])**2) **0.5
    return dis

class Camera(object):
    def __init__(self, width=1280, height=720, fps=30):
        rospy.init_node('mindvision_camera_node', anonymous=True)
        self.width = width
        self.height = height
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size=30)
        self.cam_sub = rospy.Subscriber("/vrpn_client_node/mindvision/pose", PoseStamped, self.cam_callback)
        self.car_sub = rospy.Subscriber("/vrpn_client_node/car_light/pose", PoseStamped, self.car_callback)
        # initialize camera parameters
        self.DevList = []
        self.hCamera = 0
        self.FrameBufferSize = 0
        self.pFrameBuffer = None
        self.shutter = [10000, 5000, 2500, 1250, 625, 312, 156, 78, 39, 19, 9, 5, 2, 1]
        self.fps = fps
        # paramatars from calibration
        self.cam_R = np.zeros([3,3])
        self.cam_pose = np.array([0, 0, 0])
        self.car_R = np.zeros([3,3])
        self.car_pose = np.array([0, 0, 0])
        self.exposure = 312
        self.k = 0
        self.b = 0
        self.pixel_sum = 0
        self.pixel_loc = [0, 0]
        # store data
        self.true_data = {'time':[], 'cam_pose':[], 'cam_R':[], 'car_pose':[], 'car_R':[], 'true_dis':[], 'calculated_dis':[], 'cam_exp':[], 'val':[], 'loc':[]}
        
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

        # 手动曝光，曝光时间自适应
        mvsdk.CameraSetAeState(self.hCamera, 0)
        

        # 让SDK内部取图线程开始工作
        mvsdk.CameraPlay(self.hCamera)

        # 计算RGB buffer所需的大小，这里直接按照相机的最大分辨率来分配
        self.FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)

        # 分配RGB buffer，用来存放ISP输出的图像
        # 备注：从相机传输到PC端的是RAW数据，在PC端通过软件ISP转为RGB数据（如果是黑白相机就不需要转换格式，但是ISP还有其它处理，所以也需要分配这个buffer）
        self.pFrameBuffer = mvsdk.CameraAlignMalloc(self.FrameBufferSize, 16)

        # initialize exposure time
        self.exposure_adjustment()

        
    
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

    def cam_callback(self, msg):
        # vicon消息
        global M_tool2vicon
        self.cam_R = get_matrix_from_quaternion(msg.pose.orientation)
        self.cam_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        M_tool2vicon = get_homogenious(msg.pose.orientation, msg.pose.position)
        # 例如，打印vicon的位置信息
        # print("Received vicon position:", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def car_callback(self, msg):
        # vicon消息
        global M_car2vicon
        self.car_R = get_matrix_from_quaternion(msg.pose.orientation)
        self.car_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        M_car2vicon = get_homogenious(msg.pose.orientation, msg.pose.position)
        # 例如，打印vicon的位置信息
        # print("Received vicon position:", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def calculate_distance(self, pixel_sum):
        dis = ((pixel_sum/self.exposure - self.b)/self.k)**0.5
        return dis

    def exposure_adjustment(self, low = 1, high = 100000):
        target_pixel_value = 150

        exposure_time = (low + high) // 2
        mvsdk.CameraSetExposureTime(self.hCamera, exposure_time)
        frame = self.get_frame()
        max_pixel = np.max(frame)
        best_diff = abs(target_pixel_value - max_pixel)
        best_exposure_time = exposure_time
        best_pixel = max_pixel

        

        while low <= high:
            exposure_time = (low + high) // 2
            mvsdk.CameraSetExposureTime(self.hCamera, exposure_time)
            frame = self.get_frame()
            frame = self.get_frame()
            if frame is None:
                continue
            max_pixel = np.max(frame)
            diff = abs(max_pixel - target_pixel_value)
            

            # 更新最佳差异和最佳曝光时间
            if diff < best_diff:
                best_diff = diff
                best_exposure_time = exposure_time
                best_pixel = max_pixel

            # 如果中间元素的值小于目标值，则移动到后半部分
            if max_pixel < target_pixel_value:
                low = exposure_time + 1
            # 如果中间元素的值大于目标值，则移动到前半部分
            elif max_pixel > target_pixel_value:
                high = exposure_time - 1
            else:
                break

            print('exposure_time = ', exposure_time, 'max_pixel = ', max_pixel, 'diff = ', diff)
            print('best_exposure_time = ', best_exposure_time, 'best_pixel = ', best_pixel, 'best_diff = ', best_diff)

        print(f"Best exposure time: {best_exposure_time} us with max pixel value: {best_pixel} and min diff: {best_diff}")    


        # 微调曝光时间
        self.exposure = best_exposure_time

    def mask(self, frame):
        # 计算图像的平均像素值
        mean_val = cv2.mean(frame)[0]

        # 设置阈值为图像平均像素值的2倍
        threshold = 2 * mean_val

        # 应用阈值来创建蒙版
        _, mask = cv2.threshold(frame, threshold, 255, cv2.THRESH_BINARY)

        # 应用蒙版到原图，这里使用cv2.bitwise_and函数
        masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

        # 找到蒙版中所有非零像素的坐标
        mask_coords = np.column_stack(np.where(mask > 0))

        # !need to be improved!!!!!!!!!!!! geometric center may not be the light's position if the light source are influenced by another light source
        if mask_coords.size > 0:
            center_x = int(np.mean(mask_coords[:, 1]))
            center_y = int(np.mean(mask_coords[:, 0]))
            print("蒙版的中心坐标为:", (center_x, center_y))
            self.pixel_sum = cv2.sumElems(masked_frame)[0]
            self.pixel_loc = [center_x, center_y]
        else:
            print("蒙版中没有非零像素，无法确定中心。")

        

    def record_true(self, frame):
        self.mask(frame)
        # calculated_dis = self.calculate_distance(self.pixel_sum)
        print(rospy.Time.now().to_nsec())
        t = int(str(rospy.Time.now().to_nsec()))

        # the structure of the data is : a list stores the data in following way: [time, camera position from vicon, camera rotation from vicon, car position from vicon, car rotation from vicon, camera exposure time, the sum of pixel value, the location of the light source in the frame]
        self.true_data['time'].append(t)
        self.true_data['cam_pose'].append(self.cam_pose)
        self.true_data['cam_R'].append(self.cam_R)
        self.true_data['car_pose'].append(self.cam_pose)
        self.true_data['car_R'].append(self.cam_R)
        self.true_data['true_dis'] .append(record_distance(self.cam_pose, self.car_pose))
        # self.true_data[calculated_dis'].append(calculated_dis)
        self.true_data['cam_exp'].append(self.exposure)
        self.true_data['val'].append(self.pixel_sum)
        self.true_data['loc'].append(self.pixel_loc)
        
        
    def release(self):
        # 关闭相机
        mvsdk.CameraUnInit(self.hCamera)

	    # 释放帧缓存
        mvsdk.CameraAlignFree(self.pFrameBuffer)
    


if __name__ == '__main__':
    cam = Camera()
    folder_name = input('input the folder name:')
    image_dir = f"/home/chenhaoyu/IROS_workspace/images/{folder_name}/"
    pose_dir = f"/home/chenhaoyu/IROS_workspace/images/{folder_name}/data.mat"
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

            cam.record_true(frame)

            if np.max(frame) >= 200 or np.max(frame) <= 100:
                cam.exposure_adjustment()

            # cam.publish_frame(frame)  # Publish the image as a ROS topic

            if cv2.waitKey(1) & 0xFF in [ord('q'), 27]:
                break
                
            r.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        cam.release()
        cv2.destroyAllWindows()
        for key, value in cam.true_data.items():
            cam.true_data[key] = np.array(value)
        savemat(pose_dir, cam.true_data)
        print('save file successfully')
