#!/usr/bin/env python3
import copy
from threading import Thread
import rclpy
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets # type: ignore
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2

class sunrise_controller(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        self.vision_subscriber_ = self.create_subscription(PerceptionTargets, "hobot_dnn_detection", self.vision_callback_, 10)
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 5)
        self.command_publisher_ = self.create_publisher(Int32, "sign4return", 5)
        self.command_subcriber_ = self.create_subscription(Int32, "sign4return", self.command_callback_, 10)
        # test
        self.cv_bridge_ = CvBridge()

        self.obs_lists = []

        self.move_cmd    = Twist()
        self.command_cmd = Int32()

        self.detect_qr = False
        self.trace_qr  = False
        self.go_des    = False
        self.scan_qr_success = False

        self.qr_area_threshold           = 3700
        self.parking_sign_area_threshold = 4000
        self.trace_speed                 = 0.15
        self.follow_angular_ratio        = 1.0

        self.work_timer = self.create_timer(0.004, self.timer_work_)

        self.spin_thread = Thread(target=self.spin_task_)
        self.spin_thread.start()
        
    def command_callback_(self, msg):
        if 5 == msg.data:
            self.scan_qr_success = True
            print("二维码成功识别")


    def set_patrol_mode(self, mode=1):
        if mode == 1:
            self.command_cmd.data = 6
        elif mode == 0:
            self.command_cmd.data = 5
        if mode == 2:
            self.command_cmd.data = 7
        self.command_publisher_.publish(self.command_cmd)

    def detect_whether_coming_in_qr_range(self):
        self.detect_qr = True
        while self.detect_qr == True:
            pass
        print("进入二维码识别范围")

    def trace_qr_central_point(self):
        self.set_patrol_mode(2)
        self.trace_qr = True
        while self.trace_qr == True:
            pass
        print("完成二维码识别")

    def go_to_destination(self):
        self.go_des = True
        #print("开始")
        while self.go_des == True:
            pass


    def timer_work_(self):
        if self.detect_qr == True:
            for ob in self.obs_lists:
                if ob['Type'] == "qr_code":
                    if ob['Area'] > self.qr_area_threshold:
                        self.detect_qr = False
            return
        
        if self.trace_qr == True:
            if self.scan_qr_success == True:
                self.cmd_vel.publish(Twist())
                self.trace_qr = False
                return
            for ob in self.obs_lists:
                if ob['Type'] == "qr_code":
                    # 该段仿造寻线节点
                    x = ob['CentralPoint'][0]
                    y = ob['CentralPoint'][1]
                    temp = x - 320.0
                    if (-20.0 < x) and(x < 0.0):
                        temp = -20.0
                    elif (x > 0.0) and (x < 20.0):
                        temp = 20.0
                    self.move_cmd.linear.x = self.trace_speed
                    self.move_cmd.angular.z = -self.follow_angular_ratio * temp / 150.0 * y / 224.0
                    self.cmd_vel.publish(self.move_cmd)
                    print(self.move_cmd)
                return
        
        if self.go_des == True:
            print("进来")
            print(self.obs_lists)
            for ob in self.obs_lists:
                print(ob['Type'])
                if ob['Type'] == "parking_sign":
                    print("开始")
                    print(ob['Area'])
                    if ob['Area'] > self.parking_sign_area_threshold:
                        print("结束")
                        self.set_patrol_mode(0)
                        self.go_des = False
            return

    def vision_callback_(self, msg):
        """ 视觉回调函数 """
        obs_lists = []
        ob = {'Type': '', 'CentralPoint': [], 'Area': 0} # 类型、中心点坐标、面积
        #print(msg.targets)
        if 0 != len(msg.targets):
            for i in range(len(msg.targets)):
                ob['CentralPoint'].clear()
                if msg.targets[i].type == "qr_code\r": 
                    ob['Type'] = "qr_code"
                    ob['CentralPoint'].append(msg.targets[i].rois[0].rect.x_offset + msg.targets[i].rois[0].rect.height/2)
                    ob['CentralPoint'].append(msg.targets[i].rois[0].rect.y_offset + msg.targets[i].rois[0].rect.width/2)
                    ob['Area'] = msg.targets[i].rois[0].rect.height * msg.targets[i].rois[0].rect.width
                elif msg.targets[i].type == "parking_sign\r":
                    ob['Type'] = "parking_sign"
                    ob['CentralPoint'].append(msg.targets[i].rois[0].rect.x_offset + msg.targets[i].rois[0].rect.height/2)
                    ob['CentralPoint'].append(msg.targets[i].rois[0].rect.y_offset + msg.targets[i].rois[0].rect.width/2)
                    ob['Area'] = msg.targets[i].rois[0].rect.height * msg.targets[i].rois[0].rect.width

                # 得到原始数据
                obs_lists.append(copy.deepcopy(ob)) # 深拷贝
        #print(obs_lists)
        self.obs_lists.clear()
        self.obs_lists = copy.deepcopy(obs_lists)
        #print(self.obs_lists)



    def spin_task_(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = sunrise_controller("sunrise_controller")  # 新建一个节点
        # rclpy.spin(node)
    # node.set_patrol_mode(1)
    node.detect_whether_coming_in_qr_range()
    print("面积阈值已到")
    node.trace_qr_central_point()
    #node.go_to_destination()
    print("已停车")

    while 1:
        pass
    # rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy


if __name__ == '__main__':
    main()
