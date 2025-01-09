#!/usr/bin/env python3
"""
Copyright (c) 2024 WindyLab of Westlake University, China
All rights reserved.

This software is provided "as is" without warranty of any kind, either
express or implied, including but not limited to the warranties of
merchantability, fitness for a particular purpose, or non-infringement.
In no event shall the authors or copyright holders be liable for any
claim, damages, or other liability, whether in an action of contract,
tort, or otherwise, arising from, out of, or in connection with the
software or the use or other dealings in the software.
"""

import json
from sensor_msgs.msg import Joy


import time
import rospy
import os
from paho.mqtt import client as mqtt_client


entity_id = 3 # car's ID

def rich_print(title, content):
    print(f"{title}: {content}")
#
# class Entity:
#
#     def __init__(self):
#         self.yaw = 0


class Controller():
    def __init__(self):
        self.joy_input = {'x':0,'y':0,'theta':0}
        try:
            rospy.init_node("omni_engine", anonymous=True)
        except rospy.exceptions.ROSException:
            pass

        self.mqtt_client = self.start_up_mqtt_thread()
        self.last_joy_input_time = rospy.Time.now()
        self.last_buttons = []
        self.last_axes = []
        self._entities = []

        self.joy_subscriber = rospy.Subscriber("/joy", Joy, self.joy_callback)

    def start_up_mqtt_thread(self):
        broker_ip = "10.0.2.66"
        port = 1883
        keepalive = 60
        client_id = f"{self.__class__.__name__}"

        try:
            broker = os.environ["REMOTE_SERVER"]
        except KeyError:
            broker = broker_ip

        net_status = -1
        while net_status != 0:
            net_status = os.system(f"ping -c 4 {broker}")
            time.sleep(2)

        mqtt_client_instance = MqttClientThread(
            broker=broker, port=port, keepalive=keepalive, client_id=client_id
        )
        return mqtt_client_instance

    def joy_callback(self, joy_msg):
        # 检测按键上升沿
        if not self.last_buttons:
            self.last_buttons = joy_msg.buttons
        if not self.last_axes:
            self.last_axes = joy_msg.axes

        current_buttons = joy_msg.buttons
        current_axes = joy_msg.axes

        def button_pressed(btn_idx):
            return (current_buttons[btn_idx] == 1 and (
                    len(self.last_buttons) > btn_idx and self.last_buttons[btn_idx] == 0))

        # A 按键：切换锁定和解锁
        if button_pressed(0):  # A按钮
            # self.is_locked = not self.is_locked
            pass
        # X 按键：切换Debug模式
        if button_pressed(2):  # X按钮
            pass
        # Y 按键：切换LED显示
        if button_pressed(3):  # Y按钮
            pass

        def rt_pressed():
            return (self.last_axes and len(self.last_axes) > 5 and current_axes[5] < 0.5 and self.last_axes[5] > 0.5)

        if rt_pressed():
            pass
        # 使用手柄的轴控制小车速度（若未锁定则发送控制信号）
        # 左摇杆Y轴：axes[1], 左摇杆X轴：axes[0]
        # 右摇杆X轴：axes[3]控制转向
        self.joy_input["x"] = joy_msg.axes[1] * -0.25
        self.joy_input["y"] = joy_msg.axes[0] * -0.25
        self.joy_input["theta"] = joy_msg.axes[3] * 2
        self.last_joy_input_time = rospy.Time.now()

        self.apply_joy_control()
        self.last_buttons = current_buttons
        self.last_axes = current_axes

    def control_velocity(self, desired_velocity, entity_id = 3, dt=None):
        json_msg = desired_velocity
        json_str = json.dumps(json_msg)
        self.mqtt_client.publish(
            f"/VSWARM{entity_id}_robot/motion", json_str.encode("utf-8")
        )

    def apply_joy_control(self):
        self.control_velocity(self.joy_input)


class MqttClientThread:
    def __init__(self, broker, port, keepalive, client_id):
        self.broker = broker  # MQTT代理服务器地址
        self.port = port
        self.keepalive = keepalive
        self.reconnect_interval = 1
        self.client_id = client_id
        self.client = self.connect_mqtt()
        self.client.loop_start()

    def connect_mqtt(self):
        """连接MQTT代理服务器"""

        def on_connect(client, userdata, flags, rc):
            """连接回调函数"""
            if rc == 0:
                rich_print(title="Connect to MQTT", content="Connected to MQTT OK!")
            else:
                rich_print(
                    title="Connect to MQTT",
                    content=f"Failed to connect, return code {rc}",
                )

        client = mqtt_client.Client(self.client_id)
        client.on_connect = on_connect
        try:
            client.connect(self.broker, self.port, self.keepalive)
        except Exception as e:
            rich_print(title="Connect to MQTT", content=f"Connection error: {e}")
        return client

    def publish(self, topic, msg):
        """发布消息到指定主题"""
        result = self.client.publish(topic, msg)
        status = result[0]
        if status == 0:
            pass
        else:
            print(f"Failed to send message to topic {topic}")

    def run(self):
        """启动MQTT客户端"""
        try:
            self.client.loop_forever()
        except Exception as e:
            print(f"Error in MQTT loop: {e}")
            time.sleep(self.reconnect_interval)
            try:
                self.client.loop_forever()
            except Exception as reconnect_error:
                print(f"Failed to reconnect: {reconnect_error}")
                # 再次等待一段时间后继续循环尝试
                time.sleep(self.reconnect_interval)
            # 继续循环，如果仍无法连接，继续捕获异常并处理

if __name__ == '__main__':
    controller = Controller()
    rospy.spin()
