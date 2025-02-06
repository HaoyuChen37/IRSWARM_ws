import cv2
import numpy as np
import logging

# 配置日志输出到文件和终端
logging.basicConfig(level=logging.INFO,
                    format="%(message)s",
                    handlers=[
                        logging.FileHandler("output.log", mode="a"),
                        logging.StreamHandler()
                    ])

def mask(frame):
    # 计算图像的平均像素值
    global pixel_sum, mask2, center_x, center_y, i
    frame_ave_value = cv2.mean(frame)[0]

    # 设置阈值为图像平均像素值的一定倍数
    threshold = 2 * frame_ave_value
    _, mask1 = cv2.threshold(frame, threshold, 255, cv2.THRESH_BINARY)

    # # 确保 mask 是 8 位单通道图像
    if len(mask1.shape) == 3:  # 如果 mask 是三通道，转换为单通道
        mask1 = cv2.cvtColor(mask1, cv2.COLOR_BGR2GRAY)

    # 查找蒙版中的轮廓（即光源区域）
    contours, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centers = []
    pixel_sums = []
    # # 使用开运算减少噪声
    # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    # mask = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, kernel)

    # 如果没有轮廓，直接输出 pixel_sum 为 0(既没有vicon也没有光源）
    if not contours:
        # print("No contours found. Pixel Sum: 0")
        pixel_sum = 0  # 没有光源但有vicon
        pixel_sums.append(pixel_sum)
        logging.info("No contours found. Pixel Sum: 0")
        return pixel_sums


    elif len(contours)==1:
        for i, contour in enumerate(contours):

            contour_area = cv2.contourArea(contour)  # 计算轮廓的面积

            # 计算每个亮点的矩（moments），用于计算中心
            moments = cv2.moments(contour)
            center_x = 0
            center_y = 0
            if moments["m00"] != 0:  # 避免除以零（像素个数不为0）
                center_x = int(moments["m10"] / moments["m00"])
                center_y = int(moments["m01"] / moments["m00"])
                centers.append((center_x, center_y))
            if center_y>150:
                if contour_area >= 80:#不太可能出现
                    pixel_sum = 1  # 没有光源但有vicon
                    logging.info(f"Pixel Sum02{i + 1}:{pixel_sum} {contour_area}")
                    pixel_sums.append(pixel_sum)

                elif 2 < contour_area < 80.0:  # 忽略过小的噪声且只处理面积小于80的轮廓
                    # 计算每个亮点的像素值总和
                    mask2 = np.zeros_like(frame, dtype=np.uint8)

                    if len(mask2.shape) == 3:  # 如果 mask 是三通道，转换为单通道
                        mask2 = cv2.cvtColor(mask2, cv2.COLOR_BGR2GRAY)

                    # 绘制轮廓区域到掩码上
                    cv2.drawContours(mask2, [contour], -1, (255, 255, 255), thickness=cv2.FILLED, )
                    # print(f"Pixel Sum0")
                    # 计算该区域的像素值总和
                    masked_image = cv2.bitwise_and(frame, frame, mask=mask2)
                    pixel_sum = cv2.sumElems(masked_image)[0]
                    # print(f"Pixel Sum1{i+1}: {pixel_sum}")
                    logging.info(f"Pixel Sum01{i + 1}: {pixel_sum} {contour_area}")
                    pixel_sums.append(pixel_sum)
                else:#位置较低的噪声（窗户）
                    pixel_sum = 2
                    # print(f"Pixel Sum3{i + 1}: {contour_area}")
                    logging.info(f"Pixel Sum03{i + 1}: {pixel_sum} {contour_area}")
                    pixel_sums.append(pixel_sum)
            else:#vicon或位置较高的噪声（窗户）
                pixel_sum = 1  # 没有光源但有vicon
                logging.info(f"Pixel Sum04{i + 1}: {contour_area}")
                pixel_sums.append(pixel_sum)
        # print(f"亮点中心坐标{i + 1}: ({center_x}, {center_y});Pixel Sum: {pixel_sum}")
        # # 可视化蒙版和结果
        # cv2.imshow("Mask1", mask1)
        # cv2.imshow("Frame with Centers", frame)
        # cv2.imshow("Mask2", mask2)
        # cv2.waitKey(10000)
        return pixel_sums
    else:
        for i, contour in enumerate(contours):
            # print(f"轮廓 {i + 1}: {contour}")
            # print(f"{i+1},contour")
            contour_area = cv2.contourArea(contour)  # 计算轮廓的面积

             # 计算每个亮点的矩（moments），用于计算中心
            moments = cv2.moments(contour)
            center_x = 0
            center_y = 0
            if moments["m00"] != 0:  # 避免除以零（像素个数不为0）
                center_x = int(moments["m10"] / moments["m00"])
                center_y = int(moments["m01"] / moments["m00"])
                centers.append((center_x, center_y))

            if center_y>150:

                if contour_area>=80:#不太可能出现
                    # pixel_sum = 1     #没有光源但有vicon
                    # print(f"Pixel Sum2{i+1}: {contour_area}")
                    logging.info(f"Pixel Sum2{i+1}: {contour_area}")

                elif 2<contour_area < 80.0:  # 忽略过小的噪声且只处理面积小于50的轮廓
                    # 计算每个亮点的像素值总和
                    mask2 = np.zeros_like(frame, dtype=np.uint8)

                    if len(mask2.shape) == 3:  # 如果 mask 是三通道，转换为单通道
                        mask2 = cv2.cvtColor(mask2, cv2.COLOR_BGR2GRAY)

                    # 绘制轮廓区域到掩码上
                    cv2.drawContours(mask2, [contour], -1, (255, 255, 255), thickness=cv2.FILLED, )
                    # print(f"Pixel Sum0")
                    # 计算该区域的像素值总和
                    masked_image = cv2.bitwise_and(frame, frame, mask=mask2)
                    pixel_sum = cv2.sumElems(masked_image)[0]
                    # print(f"Pixel Sum1{i+1}: {pixel_sum}")
                    logging.info(f"Pixel Sum1{i+1}: {pixel_sum} {contour_area}")
                    pixel_sums.append(pixel_sum)
                else:
                    # pixel_sum3 = 0
                    # print(f"Pixel Sum3{i + 1}: {contour_area}")
                    logging.info(f"Pixel Sum3{i + 1}: {pixel_sum} {contour_area}")
            else:
                logging.info(f"Pixel Sum05{i + 1}:{pixel_sum} {contour_area}")


        #     logging.info(f"Pixel Sum:{pixel_sums}")
        #
        #     print(f"亮点中心坐标{i+1}: ({center_x}, {center_y});Pixel Sum: {pixel_sum}")
        #
        #     # 可将亮点中心绘制在原图上
        #     radius = 5
        #     offset_x = -10#-20
        #     offset_y = -30#8
        #     # # 在图像上绘制圆形框（绿色，2 像素宽）
        #     cv2.circle(frame, (center_x, center_y), radius, (0, 255, 0), 2)
        #     id_position = (center_x + radius + offset_x, center_y - radius - offset_y)
        #     # 设置 ID 显示位置，稍微向右偏移 `offset`，避免与圆形框重叠
        #     cv2.putText(frame, f"ID:{i + 1}", id_position,
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        # # 可视化蒙版和结果
        # cv2.imshow("Mask1", mask1)
        # cv2.imshow("Frame with Centers", frame)
        # cv2.imshow("Mask2", mask2)
        # cv2.waitKey(10000)
        #如果 pixel_sums 为空，则赋值为 [0]
        if not pixel_sums:
            pixel_sums = [0]
            print(f"亮点中心坐标{i + 1}: ({center_x}, {center_y});Pixel Sum: {pixel_sum}")
        # 检查 pixel_sums 中数的个数
        if len(pixel_sums) == 2:
            logging.info("Error in load pixe_sums")
        return pixel_sums



if __name__=="__main__":

    image_path = r"D:\Data\cam1\datanew\0116125804600038.png"
    # image_path = r"D:\test_vscode\pycharmproject\data\vicon.png"
    # 0116125601699770  0116125603166391  0116125604899799  0116125632499739 0116125905499759
    frame = cv2.imread(image_path)  # image本是RAW格式单通道，但是cv2.read的函数会默认将它读成三通道图像结果

    # print("frame shape:", frame.shape, "dtype:", frame.dtype)
    # 确保图像加载成功
    if frame is not None:
        # cv2.imshow("Frame with Centers", frame)
        # cv2.waitKey(10000)
        mask(frame)
    else:
        print("无法加载图像。")
