# multi-light-detecting system
## install camera SDK

## Create ROS workspace
创建工作空间并初始化
```bash
mkdir -p LightSwarm_ws/src
cd LightSwarm_ws
catkin_make
```
进入 src 创建 ros 包并添加依赖
```bash
cd src
catkin_create_pkg mv roscpp rospy std_msgs
```
进入 ros 包添加 scripts 目录并编辑 python 文件
```bash
cd mv
mkdir scripts
```
复制文件到script文件夹下面，添加可操作权限
```bash
cd scripts
chmod +x mv_con.py
```
调整缩进
```bash
sudo pip install autopep8
autopep8 -i /home/nvidia/LightSwarm_ws/src/mv/scripts/control.py
```
```bash
mkdir -p LightSwarm_ws/src
cd LightSwarm_ws
catkin_make

cd src
catkin_create_pkg mv roscpp rospy std_msgs

cd mv
mkdir scripts
```
复制文件到script文件夹下面，添加可操作权限
```bash
chmod +x mv_con.py

sudo pip install autopep8
autopep8 -i /home/nvidia/LightSwarm_ws/src/mv/scripts/control.py
```

nvidia-docker run -it --rm     --name llm-code     -v /home/nvidia/docker/code_llm_ws:/catkin_ws     --workdir /catkin_ws     --network host     llm_simulator


手柄控制需要的一些依赖
```bash
sudo apt-get install ros-noetic-joy
pip install paho-mqtt
```
