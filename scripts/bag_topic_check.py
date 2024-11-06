import rosbag

bag = rosbag.Bag(r'/home/chenhaoyu/2024-10-11-13-14-06.bag')


# 遍历bag文件
for topic, msg, t in bag.read_messages():

    if topic == '/camera/color/image_raw':
        print('img exists')
    if topic == '/vrpn_client_node/mindvision/pose' :
        print('position exists')
