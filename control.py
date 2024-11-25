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

class Camera(object):
    def __init__(self, width=1280, height=720, fps=30):
        rospy.init_node('mindvision_camera_node', anonymous=True)
        self.width = width
        self.height = height
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size=10)
        # initialize camera parameters
        self.DevList = []
        self.hCamera = 0
        self.FrameBufferSize = 0
        self.pFrameBuffer = None
        self.shutter = [10000, 5000, 2500, 1250, 625, 312, 156, 78, 39, 19, 9, 5, 2, 1]
        self.fps = fps
        # paramatars from calibration
        self.k
        self.b
        # store data
        self.data = []
        
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

    def record(self, frame):
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
        else:
            print("蒙版中没有非零像素，无法确定中心。")

        pixel_sum = cv2.sumElems(masked_frame)[0]

        t = rospy.Time.now()

        # the structure of the data is : a list stores the data in following way: [time, camera position from vicon, camera position from vicon, camera exposure time, the sum of pixel value, the location of the light source in the frame]
        self.data.append([t, None, None, self.exposure, pixel_sum, [center_x, center_y]])
        
        
    def release(self):
        # 关闭相机
        mvsdk.CameraUnInit(self.hCamera)

	    # 释放帧缓存
        mvsdk.CameraAlignFree(self.pFrameBuffer)
    


if __name__ == '__main__':
    cam = Camera()
    # 修改为Linux风格的路径
    image_dir = r"/home/chenhaoyu/IROS_workspace/images/exp50_Ap4_5_1_5m/"
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

            if np.max(frame) >= 200 or np.max(frame) <= 100:
                cam.exposure_adjustment()

            cam.publish_frame(frame)  # Publish the image as a ROS topic

            if cv2.waitKey(1) & 0xFF in [ord('q'), 27]:
                break
                
            r.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        cam.release()
        cv2.destroyAllWindows()
