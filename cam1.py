#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from datetime import datetime
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge
import numpy as np
import mvsdk
import platform
from scipy.io import savemat
import tf
from datetime import datetime
from mv.msg import _LightInfo, _LightsInfo


def get_homogenious(quaternion, position):
    # 将四元数转换为欧拉角，返回matrix
    R = tf.transformations.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    t = np.array([position.x, position.y, position.z])
    T = np.zeros([4, 4])
    T[0:3, 0:3] = R[0:3, 0:3]
    T[0, 3] = t[0]
    T[1, 3] = t[1]
    T[2, 3] = t[2]
    T[3, 3] = 1
    return T


def get_matrix_from_quaternion(quaternion):
    matrix = tf.transformations.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    matrix = matrix[:3, :3]
    return matrix


def get_euler_from_quaternion(quaternion):
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    euler = [roll, pitch, yaw]
    return euler


def record_distance(pose1, pose2):
    dis = ((pose1[0] - pose2[0]) ** 2 + (pose1[1] - pose2[1]) ** 2 + (pose1[2] - pose2[2]) ** 2) ** 0.5
    return dis


class Camera(object):
    def __init__(self, width=1280, height=720, fps=30):
        rospy.init_node('mindvision_camera_node1', anonymous=True)
        self.width = width
        self.height = height
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size=30)
        self.cam_sub = rospy.Subscriber("/vrpn_client_node/IRSWARM1/pose", PoseStamped, self.cam_callback)
        # initialize camera parameters
        self.DevList = []
        self.hCamera = 0
        self.FrameBufferSize = 0
        self.pFrameBuffer = None
        self.shutter = [10000, 5000, 2500, 1250, 625, 312, 156, 78, 39, 19, 9, 5, 2, 1]
        self.fps = fps
        # paramatars from calibration
        self.cam_R = np.zeros([3, 3])
        self.cam_euler = np.array([0, 0, 0])
        self.cam_pose = np.array([0, 0, 0])
        self.car_R = np.zeros([3, 3])
        self.car_euler = np.array([0, 0, 0])
        self.car_pose = np.array([0, 0, 0])
        self.exposure = 312
        self.k = 0
        self.b = 0
        self.pixel_sum = []
        self.pixel_loc = []
        self.frame_ave_value = 0
        self.frame_max_value = 0
        # store data
        self.true_data = {'time': [], 'cam_pose': [], 'cam_R': [], 'cam_euler': [], 'car_pose': [], 'car_R': [],
                          'car_euler': [], 'true_dis': [], 'calculated_dis': [], 'cam_exp': [], 'val': [], 'loc': [],
                          'frame_ave_value': []}

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
        # self.exposure_adjustment()
        mvsdk.CameraSetExposureTime(self.hCamera, self.exposure)

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
        frame = frame.reshape(
            (FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))

        frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
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
        self.cam_euler = get_euler_from_quaternion(msg.pose.orientation)
        self.cam_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        M_tool2vicon = get_homogenious(msg.pose.orientation, msg.pose.position)
        # 例如，打印vicon的位置信息
        # print("Received vicon position:", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def car_callback(self, msg):
        # vicon消息
        global M_car2vicon
        self.car_R = get_matrix_from_quaternion(msg.pose.orientation)
        self.car_euler = get_euler_from_quaternion(msg.pose.orientation)
        self.car_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        M_car2vicon = get_homogenious(msg.pose.orientation, msg.pose.position)
        # 例如，打印vicon的位置信息
        # print("Received vicon position:", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def calculate_distance(self, pixel_sum):
        dis = ((pixel_sum / self.exposure - self.b) / self.k) ** 0.5
        return dis

    def exposure_adjustment(self, low=1, high=100000):
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

        print(
            f"Best exposure time: {best_exposure_time} us with max pixel value: {best_pixel} and min diff: {best_diff}")

        # 微调曝光时间
        self.exposure = best_exposure_time

    def mask(self,frame):

        # 计算图像的平均像素值
        self.frame_ave_value = cv2.mean(frame)[0]

        # 设置阈值为图像平均像素值的一定倍数
        threshold = 2 * self.frame_ave_value
        _, mask1 = cv2.threshold(frame, threshold, 255, cv2.THRESH_BINARY)

        # # 确保 mask 是 8 位单通道图像
        if len(mask1.shape) == 3:  # 如果 mask 是三通道，转换为单通道
            mask1 = cv2.cvtColor(mask1, cv2.COLOR_BGR2GRAY)

        # 多光源目标检测
        # # 使用开运算减少噪声
        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        # mask = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, kernel)

        # 查找蒙版中的轮廓（即光源区域）
        contours, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centers = []
        # print(contours)

        if contours:
            for contour in contours:
                if cv2.contourArea(contour) > 1:  # 忽略面积过小的噪声
                    # 计算每个亮点的矩（moments），用于计算中心
                    moments = cv2.moments(contour)
                    if moments["m00"] != 0:  # 避免除以零
                        center_x = int(moments["m10"] / moments["m00"])
                        center_y = int(moments["m01"] / moments["m00"])
                        centers.append([center_x, center_y])
                        print(f"亮点中心坐标: ({center_x}, {center_y})")

        # 可将亮点中心绘制在原图上
        for idx, [center_x, center_y] in enumerate(centers):
            # 在图像上绘制圆形框（假设半径为 10）
            radius = 5
            # offset_x = -20
            # offset_y = 8
            # text_positions = []
            # # 在图像上绘制圆形框（绿色，2 像素宽）
            cv2.circle(frame, (center_x, center_y), radius, (0, 255, 0), 2)

            # # 设置 ID 显示位置，稍微向右偏移 `offset`，避免与圆形框重叠
            # # id_position = (center_x + radius + offset_x, center_y - radius - offset_y)
            # # cv2.putText(frame, f"ID:{idx + 1}", id_position,
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # 提取圆形框内的像素值区域
            mask2 = np.zeros(frame.shape[:2], dtype=np.uint8)  # 创建一个与图像大小相同的黑色遮罩
            cv2.circle(mask2, (center_x, center_y), radius, 255, -1)  # 在遮罩上绘制白色圆形

            # 通过遮罩提取圆形区域的像素值
            masked_region = cv2.bitwise_and(frame, frame, mask=mask2)

            # 计算该区域的像素值总和
            self.pixel_sum = np.sum(masked_region)

            # 打印像素值总和（这里按通道分开显示，总和是 RGB 各个通道的总和）
            #print(f"ID: {idx + 1}, Pixel Sum: {pixel_sum}")

        # 可视化蒙版和结果
        # cv2.imshow("Mask1", mask1)
        # cv2.imshow("Mask2", mask2)
        #cv2.imshow("Frame with Centers", frame)
        #cv2.waitKey(10000)

    def record_true(self, frame):
        self.mask(frame)
        # calculated_dis = self.calculate_distance(self.pixel_sum)
        print(rospy.Time.now().to_nsec())
        t = int(str(rospy.Time.now().to_nsec()))

        # the structure of the data is : a list stores the data in following way: [time, camera position from vicon, camera rotation from vicon, car position from vicon, car rotation from vicon, camera exposure time, the sum of pixel value, the location of the light source in the frame]
        self.true_data['time'].append(t)
        self.true_data['cam_pose'].append(self.cam_pose)
        self.true_data['cam_R'].append(self.cam_R)
        self.true_data['cam_euler'].append(self.cam_euler)
        self.true_data['car_pose'].append(self.cam_pose)
        self.true_data['car_R'].append(self.cam_R)
        self.true_data['car_euler'].append(self.car_euler)
        self.true_data['true_dis'].append(record_distance(self.cam_pose, self.car_pose))
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
    # folder_name = input('input the folder name:')
    folder_name = '1'
    image_dir = f"Data/cam1/{folder_name}/"
    pose_dir = f"Data/cam1/{folder_name}/data.mat"
    if not os.path.exists(image_dir):
        # 在Linux中创建目录
        os.makedirs(image_dir)
        
    if not os.path.exists(pose_dir):
        # 在Linux中创建目录
        os.makedirs(pose_dir)

    try:
        r = rospy.Rate(30)  # 30hz
        cam.initialization()

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
