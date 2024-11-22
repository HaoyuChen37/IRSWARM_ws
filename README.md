# multi-light-detecting system
## install camera SDK

## Create ROS workspace
创建工作空间并初始化
```bash
mkdir -p chy_workspace/src
cd chy_workspace
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
cd scripts
touch mv_con.py
chmod +x mv_con.py
```

```bash

mkdir -p chy_workspace/src
cd chy_workspace
catkin_make

cd src
catkin_create_pkg mv roscpp rospy std_msgs

cd mv
mkdir scripts
cd scripts
touch mv_con.py
chmod +x mv_con.py
```


