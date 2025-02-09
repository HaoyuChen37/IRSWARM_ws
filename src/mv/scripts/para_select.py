from light_processing import process_frame
import cv2
if __name__=="__main__":

    image_path = f"/home/chenhaoyu/IRSWARM_ws/src/mv/scripts/0207155201770955.png"
    # image_path = r"D:\test_vscode\pycharmproject\data\vicon.png"
    # 0116125601699770  0116125603166391  0116125604899799  0116125632499739 0116125905499759
    frame = cv2.imread(image_path)  # image本是RAW格式单通道，但是cv2.read的函数会默认将它读成三通道图像结果

    # print("frame shape:", frame.shape, "dtype:", frame.dtype)
    # 确保图像加载成功
    if frame is not None:
        # cv2.imshow("Frame with Centers", frame)
        # cv2.waitKey(10000)
        print(process_frame(frame))
    else:
        print("无法加载图像。")
