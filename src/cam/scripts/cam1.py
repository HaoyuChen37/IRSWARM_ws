#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from datetime import datetime
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import mvsdk
import platform
from scipy.io import savemat
from datetime import datetime
from cam.msg import LightInfo, Cam1
from light_processing import LightLocalizer


class Camera(object):
    def __init__(self, width=1280, height=720, fps=100):
        rospy.init_node('Cam1_node', anonymous=True)
        self.width = width
        self.height = height
        self.bridge = CvBridge()
        self.light_pub = rospy.Publisher('/Cam1', Cam1, queue_size=100)
        # initialize camera parameters
        self.DevList = []
        self.hCamera = 0
        self.FrameBufferSize = 0
        self.pFrameBuffer = None
        self.shutter = [10000, 5000, 2500, 1250, 625, 312, 156, 78, 39, 19, 9, 5, 2, 1]
        self.fps = fps
        self.exposure = 312
        self.pixel_sum = []
        self.pixel_loc = []
        self.frame_ave_value = 0
        self.frame_max_value = 0
        # Initialize car_id, cam_id
        self.car_id = 1
        self.cam_id = 0
        
    def initialization(self):
        # 枚举相机
        self.DevList = mvsdk.CameraEnumerateDevice()
        nDev = len(self.DevList)
        print('nDev = ', nDev)
        if nDev < 1:
            print("No camera was found!")
            return

        for i, DevInfo in enumerate(self.DevList):
            print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))
        # n-th camera
        i = 0
        DevInfo = self.DevList[i]
        print(DevInfo)

        # 打开相机
        self.hCamera = 0
        try:
            self.hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
        except mvsdk.CameraException as e:
            print("CameraInit Failed({}): {}".format(e.error_code, e.message))
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
        self.FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (
            1 if monoCamera else 3)

        # 分配RGB buffer，用来存放ISP输出的图像
        # 备注：从相机传输到PC端的是RAW数据，在PC端通过软件ISP转为RGB数据（如果是黑白相机就不需要转换格式，但是ISP还有其它处理，所以也需要分配这个buffer）
        self.pFrameBuffer = mvsdk.CameraAlignMalloc(self.FrameBufferSize, 16)

        # initialize exposure time
        self.exposure_adjustment()
        # mvsdk.CameraSetExposureTime(self.hCamera, self.exposure)

    def get_frame(self):
        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.hCamera, 200)
        mvsdk.CameraImageProcess(self.hCamera, pRawData, self.pFrameBuffer, FrameHead)
        mvsdk.CameraReleaseImageBuffer(self.hCamera, pRawData)

        # 此时图片已经存储在pFrameBuffer中，对于彩色相机pFrameBuffer=RGB数据，黑白相机pFrameBuffer=8位灰度数据
        # 把pFrameBuffer转换成opencv的图像格式以进行后续算法处理
        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self.pFrameBuffer)
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        frame = frame.reshape(
            (FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))

        frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
        # cv2.imshow("Press q to end", frame)

        return frame

    def exposure_adjustment(self, low=1, high=100000):
        target_pixel_value = 220

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

        print(
            f"Best exposure time: {best_exposure_time} us with max pixel value: {best_pixel} and min diff: {best_diff}")

        # 微调曝光时间
        self.exposure = best_exposure_time

    def mask(self, frame, localizer, with_vicon = 0):
        if with_vicon == 0:
            # 调用 process_frame_without_vicon 方法
            self.pixel_loc, self.pixel_sum = localizer.process_frame_without_vicon(frame)
        else:
            self.pixel_loc, self.pixel_sum = localizer.process_frame_with_vicon(frame, self.car_id, self.cam_id)

        # reproject method
        lights = localizer.reproject(self.pixel_loc, self.pixel_sum, self.exposure, self.cam_id)

        lights_info = Cam1(lights=lights)
        self.light_pub.publish(lights_info)

    def release(self):
        # 关闭相机
        mvsdk.CameraUnInit(self.hCamera)

        # 释放帧缓存
        mvsdk.CameraAlignFree(self.pFrameBuffer)
    


if __name__ == '__main__':
    cam = Camera()
    # folder_name = input('input the folder name:')
    folder_name = '2'
    image_dir = f"Data/{folder_name}/cam1/"
    if not os.path.exists(image_dir):
        # 在Linux中创建目录
        os.makedirs(image_dir)

    try:
        r = rospy.Rate(100)  # 100hz
        cam.initialization()
        # 创建 LightLocalizer 类的实例
        localizer = LightLocalizer()

        while not rospy.is_shutdown():
            try:
                current_time = datetime.now().strftime('%m%d%H%M%S%f')
                frame = cam.get_frame()
                cv2.imwrite(image_dir + current_time + ".png", frame)
                if frame is None:
                    continue
            except mvsdk.CameraException as e:
                if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                    print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message))

            cam.mask(frame, localizer)

            # if np.max(frame) >= 250 or np.max(frame) <= 20:
            #     cam.exposure_adjustment()

            if cv2.waitKey(1) & 0xFF in [ord('q'), 27]:
                break

            r.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        cam.release()
        print('camera1 has closed')