#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from datetime import datetime
import rospy
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
import mvsdk
import platform
from scipy.io import savemat
from datetime import datetime
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt



def record_distance(pose1, pose2):
    dis = ((pose1[0]-pose2[0])**2 + (pose1[1]-pose2[1])**2 + (pose1[2]-pose2[2])**2) **0.5
    return dis

class Camera(object):
    def __init__(self, width=1280, height=720, fps=30):
        rospy.init_node('mindvision_camera_node1', anonymous=True)
        self.width = width
        self.height = height
        self.cam_sub = rospy.Subscriber("/vrpn_client_node/mindvision/pose", PoseStamped, self.cam_callback)
        self.light_sub = rospy.Subscriber("/vrpn_client_node/car_light/pose", PoseStamped, self.light_callback)
        # initialize camera parameters
        self.DevList = []
        self.hCamera = 0
        self.FrameBufferSize = 0
        self.pFrameBuffer = None
        self.shutter = [10000, 5000, 2500, 1250, 625, 312, 156, 78, 39, 19, 9, 5, 2, 1]
        self.fps = fps
        # paramatars from calibration
        self.cam_pose = np.array([0, 0, 0])
        self.light_pose = np.array([0, 0, 0])
        self.exposure = 312
        self.k = 0
        self.b = 0
        self.pixel_sum = 0
        self.pixel_loc = [0, 0]
        self.frame_ave_value = 0
        self.frame_max_value = 0
        # store data
        self.true_data = {'time':[], 'cam_pose':[], 'light_pose':[], 'true_dis':[], 'cam_exp':[], 'val':[], 'frame_ave_value':[]}
        
    def initialization(self):
    # 枚举相机
        self.DevList = mvsdk.CameraEnumerateDevice()
        nDev = len(self.DevList)
        if nDev < 1:
            print("No camera was found!")
            return
        
        for i, DevInfo in enumerate(self.DevList):
            print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))
        
        # calibrate n-th camera
        print(f'Now we have {nDev} cameras')
        i = int(input('Which camera are you going to calibration? please input a number in between 0~3:')) 

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

    def cam_callback(self, msg):
        # vicon消息
        self.cam_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def light_callback(self, msg):
        # vicon消息
        self.light_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

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
        frame_ave_value = cv2.mean(frame)[0]

        # gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # equalized_frame = cv2.equalizeHist(gray_frame)

        # 设置阈值为图像平均像素值的一定倍数
        threshold =2* frame_ave_value

        # 应用自适应阈值来创建蒙版
        # blockSize = 11  # 邻域大小
        # C = 2  # 常数
        _, mask = cv2.threshold(frame, threshold, 255, cv2.THRESH_BINARY)
        # mask = cv2.adaptiveThreshold(  equalized_frame, 1, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, blockSize,C)


        # # 确保 mask 是 8 位单通道图像
        if len(mask.shape) == 3:  # 如果 mask 是三通道，转换为单通道
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)


        #多光源目标检测
        # # 使用开运算减少噪声
        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 查找蒙版中的轮廓（即光源区域）
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centers = []
        print(contours)

        if contours:
            for contour in contours:
                if cv2.contourArea(contour) > 1:  # 忽略面积过小的噪声
                    # 计算每个亮点的矩（moments），用于计算中心
                    Moments = cv2.moments(contour)
                    if Moments["m00"] != 0:  # 避免除以零
                        center_x = int(Moments["m10"] / Moments["m00"])
                        center_y = int(Moments["m01"] / Moments["m00"])
                        centers.append((center_x, center_y))
                        print(f"亮点中心坐标: ({center_x}, {center_y})")



        # # 可将亮点中心绘制在原图上
        # for (center_x, center_y) in centers:
        #     # 在图像上绘制圆形框（假设半径为 10）
        #     radius = 8
        #     # 在图像上绘制圆形框（绿色，2 像素宽）
        #     cv2.circle(frame, (center_x, center_y), radius, (0, 255, 0), 2)

        # # 可视化蒙版和结果
        # cv2.imshow("Mask", mask)
        # cv2.imshow("Frame with Centers", frame)
        # cv2.waitKey(10000)
        '''
        
        self.pixel_sum = cv2.sumElems(masked_frame)[0]
        self.pixel_loc = [center_x, center_y]
        
        我需要在self里面有一个字典, 字典的键是亮点的索引 i, 值是一个字典, 包含亮点的中心坐标和像素值总和。
        '''



        

    def record_true(self, frame):
        self.mask(frame)
        # calculated_dis = self.calculate_distance(self.pixel_sum)
        print(rospy.Time.now().to_nsec())
        t = int(str(rospy.Time.now().to_nsec()))

        # the structure of the data is : a list stores the data in following way: [time, camera position from vicon, camera rotation from vicon, car position from vicon, car rotation from vicon, camera exposure time, the sum of pixel value, the location of the light source in the frame]
        self.true_data['time'].append(t)
        self.true_data['cam_pose'].append(self.cam_pose)
        self.true_data['light_pose'].append(self.light_pose)
        self.true_data['true_dis'] .append(record_distance(self.cam_pose, self.light_pose))
        # self.true_data['calculated_dis'].append(calculated_dis)
        self.true_data['cam_exp'].append(self.exposure)
        self.true_data['val'].append(self.pixel_sum)
        self.true_data['loc'].append(self.pixel_loc)
        self.true_data['frame_ave_value'].append(self.frame_ave_value)
        # self.true_data['frame_max_value'].append(self.frame_max_value)
	    
    def release(self):
        # 关闭相机
        mvsdk.CameraUnInit(self.hCamera)

	    # 释放帧缓存
        mvsdk.CameraAlignFree(self.pFrameBuffer)
    


if __name__ == '__main__':
    cam = Camera()
    folder_name = input('input the folder name:')
    image_dir = f"Data/images/{folder_name}/"
    pose_dir = f"Data/images/{folder_name}/data.mat"
    if not os.path.exists(image_dir):
        # 在Linux中创建目录
        os.makedirs(image_dir)
    
    try:
        r = rospy.Rate(30)  # 30hz
        cam.initialization()
       
        while not rospy.is_shutdown():
            try:
                current_time = datetime.now().strftime('%m%d%H%M%S%f')
                frame = cam.get_frame()
                cv2.imwrite(image_dir+current_time+".png",frame)
                if frame is None:
                    continue
            except mvsdk.CameraException as e:
                if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                    print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message) )

            cam.record_true(frame)

            # if np.max(frame) >= 200 or np.max(frame) <= 100:
            #     cam.exposure_adjustment()

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

        # 拟合参数
        # 定义模型函数
        def model_function(pixel_sum, b, k):
            return ((pixel_sum / cam.exposure - b) / k) ** 0.5

        # 准备数据
        pixel_sums = np.array(cam.true_data['val'])
        true_distances = np.array(cam.true_data['true_dis'])

        # 拟合模型
        params, covariance = curve_fit(model_function, pixel_sums, true_distances)

        # 提取拟合参数
        cam.b, cam.k = params
        print(f"Fitted parameters: b = {cam.b}, k = {cam.k}")


        # 计算拟合后的距离并比较误差
        # 使用拟合参数计算距离
        calculated_distances = model_function(pixel_sums, cam.b, cam.k)

        # 计算百分比误差
        percentage_errors = (np.abs(calculated_distances - true_distances) / true_distances) * 100
        mean_percentage_error = np.mean(percentage_errors)
        print(f"Mean percentage error: {mean_percentage_error:.2f}%")

        # 可视化结果
        plt.figure(figsize=(10, 5))
        plt.plot(true_distances, label='True Distance')
        plt.plot(calculated_distances, label='Calculated Distance')
        plt.plot(percentage_errors, label='Percentage Error', color='red')
        plt.legend()
        plt.xlabel('Sample Index')
        plt.ylabel('Distance (m) / Percentage Error (%)')
        plt.title('Comparison of True and Calculated Distances with Percentage Error')
        plt.grid(True)

        # 保存图表到文件
        plt.savefig('distance_comparison.png')
        plt.close()  # 关闭图表，避免显示