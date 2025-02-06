#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from mv.msg import LightsInfo
from geometry_msgs.msg import Twist  # 导入Twist消息类型用于速度控制

# 定义一个全局字典来存储消息数据
lights_data = {}
vel = (0, 0)
k = 1617
b = 533

def callback(data):
    rospy.loginfo("Received LightsInfo:")
    global lights_data, vel, k, b
    vel = (0, 0)
    dis = 1
    p = 1
    # 初始化一个列表来存储每个LightInfo的字典
    lights_list = []
    
    for light in data.lights:
        # 将每个LightInfo转换为字典
        light_dict = {
            'x': light.x,
            'y': light.y,
            'strength': light.strength
        }
        lights_list.append(light_dict)
        distance = (k / (light.strength - b))**0.5
        # 计算速度
        vel = (vel[0] + p * light.x * (distance - dis), vel[1] + p * light.y * (light.distance - dis))
    
    # 将列表保存到全局字典中
    lights_data['lights'] = lights_list
    
    # 打印字典内容（可选）
    
    rospy.loginfo(lights_data)
    
    # 发布速度指令
    publish_velocity(vel)

def publish_velocity(velocity):
    # 创建速度消息
    velocity_msg = Twist()
    velocity_msg.linear.x = velocity[0]
    velocity_msg.linear.y = velocity[1]
    
    # 发布速度消息到/robot/velcmd
    velocity_publisher.publish(velocity_msg)
    rospy.loginfo("Published velocity command: Linear x={}, y={}".format(velocity[0], velocity[1]))

def lights_info_subscriber():
    global velocity_publisher  # 声明为全局变量以便在callback函数中使用
    # 初始化ROS节点
    rospy.init_node('lights_info_subscriber', anonymous=True)
    
    # 创建一个发布者，发布到/robot/velcmd
    velocity_publisher = rospy.Publisher('/robot/velcmd', Twist, queue_size=10)
    
    # 创建一个订阅者，订阅名为'lights_info'的主题
    rospy.Subscriber('lights_info', LightsInfo, callback)
    
    # 保持节点运行，直到被中断
    rospy.spin()

if __name__ == '__main__':
    try:
        lights_info_subscriber()
    except rospy.ROSInterruptException:
        pass