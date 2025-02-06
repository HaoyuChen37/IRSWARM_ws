import cv2
import numpy as np

import cv2
import numpy as np

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
        pixel_sums.append(0)
        return pixel_sums

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

    return pixel_sums