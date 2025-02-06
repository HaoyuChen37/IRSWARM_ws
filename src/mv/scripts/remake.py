import logging
import os
import cv2
import numpy as np
from scipy.io import savemat, loadmat
import multi_detect

with open("output.log", "w") as file:
    file.write("")  # 清空文件内容

# 配置日志输出到文件和终端
logging.basicConfig(level=logging.INFO,
                    format="%(message)s",
                    handlers=[
                        logging.FileHandler("output.log",mode="a"),
                        logging.StreamHandler()
                    ])

# 定义文件夹路径
input_folder = r"D:\Data\cam1\datanew"  # 图片所在文件夹
# 加载原始 .mat 文件
mat_file_path = r"D:\Data\cam1\data_new.mat"
data = loadmat(mat_file_path)  # 加载原始文件内容

# 初始化存储信息的列表，用于保存到 MATLAB 文件
new_val = []

for file_name in os.listdir(input_folder):
    file_path = os.path.join(input_folder, file_name)

    # 确保只处理图片文件
    if file_name.lower().endswith(('.png')):
        logging.info(f"Processing: {file_name}")
        pixel_sums = []
        try:
            # 读取图像
            frame = cv2.imread(file_path)
            if frame is None:
                # raise ValueError(f"Cannot read image: {file_name}")
                logging.info(f"Cannot read image: {file_name}")
            # 调用 multi_detect.mask(frame)
            pixel_sums = multi_detect.mask(frame)
            logging.info(f"pixel_sums :{pixel_sums}")



            # 检查 pixel_sum 是否有效
            if pixel_sums is None:
                pixel_sum = np.array([0], dtype=np.uint64)
                # raise ValueError(f"multi_detect.mask returned None for {file_name}")
                logging.info(f"multi_detect.mask returned None for {file_name}")
            # 确保 pixel_sum 是一个数值或数组
            if isinstance(pixel_sums, np.ndarray):
                pixel_sum = np.array(pixel_sums, dtype=np.uint64)
                # pixel_sum = pixel_sums.astype(np.uint64)
                logging.info(f"Succeed storing pixel_sums:{pixel_sum}" )
            # new_val.append(pixel_sum)
            new_val.extend(pixel_sums)

        except Exception as e:
            # print(f"Error processing {file_name}: {e}")
            logging.info(f"Error processing {file_name}: {e}")
            # new_val.append(pixel_sum)
            continue

# 将 new_val 转换为 NumPy 数组
new_val = np.array([new_val], dtype=object)  # 保持与原 val 格式一致
data['val'] = new_val
# 保存更新后的数据
savemat(mat_file_path, data)  # 直接覆盖保存
print(f"Processing complete. Results saved ")

# sys.stdout = sys.__stdout__