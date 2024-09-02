#!/usr/bin/env python3
import copy
from threading import Thread
from math import copysign
import time
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
        self.point_subscriber_ = self.create_subscription(PerceptionTargets, "racing_track_center_detection", self.point_callback_, 10)
        self.image_subscriber_ = self.create_subscription(Image, 'image', self.image_callback_, 10)
        self.compressed_image_publisher_ = self.create_publisher(CompressedImage, 'compressed_image1', 10)
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 5)
        self.command_publisher_ = self.create_publisher(Int32, "/sign4return", 5)
        self.command_subcriber_ = self.create_subscription(Int32, "/sign4return", self.command_callback_, 10)

        self.obs_lists = []
        self.line_central_point = []

        self.move_cmd    = Twist()
        self.command_cmd = Int32()

        self.detect_qr                   = False
        self.trace_qr                    = False
        self.go_des                      = False
        self.scan_qr_success             = False
        self.start_patrol                = False

        # 可调参数
        self.obstacle_threshold          = [7000, 8500]
        self.qr_area_threshold           = [3700, 4500]
        self.parking_sign_area_threshold = 4000
        ## 关键参数
        self.trace_angular_ratio         = 1.0
        self.trace_liear_speed           = 0.15
        self.follow_angular_ratio        = -1.0
        self.follow_linear_speed         = 0.1
        self.avoid_angular_ratio         = 1.1
        self.avoid_linear_speed          = 0.25

        self.work_timer = self.create_timer(0.001, self.timer_work_)
        self.patrol_timer_ = self.create_timer(0.001, self.patrol_timer_work_)

        self.spin_thread = Thread(target=self.spin_task_)
        self.spin_thread.start()

    def patrol_timer_work_(self):
        obstacle_area_list = []
        direction = 1
        max_area = 0.0
        for ob in self.obs_lists:
            if ob['Type'] == 'obstacle':
                obstacle_area_list.append(ob['Area'])
        if 0 != len(obstacle_area_list):
            for ob in self.obs_lists:
                if ob['Area'] == max(obstacle_area_list):
                    if 320.0 - ob['CentralPoint'] > 0:
                        direction = 1 # 右边画圆
                    else:
                        direction = -1 # 左边画圆
                max_area = max(obstacle_area_list)

        if self.start_patrol:
            if max_area < self.obstacle_threshold[0]:
                temp = self.line_central_point[0] - 320.0
                if -20.0 < self.line_central_point[0] and self.line_central_point[0] < 0.0:
                    temp = -20.0
                elif self.line_central_point[0] > 0.0 and self.line_central_point[0] < 20.0:
                    temp = 20.0

                self.move_cmd.linear.x = self.follow_linear_speed
                self.move_cmd.angular.z = self.follow_angular_ratio * temp / 150.0 * self.line_central_point[1] / 224.0
                self.cmd_vel.publish(self.move_cmd)
            elif max_area < self.obstacle_threshold[1]:
                # 避障
                self.move_cmd.linear.x = self.avoid_linear_speed
                self.move_cmd.angular.z = copysign(self.avoid_angular_ratio, direction) / 300 * 0.4
                self.cmd_vel.publish(self.move_cmd)
                time.sleep(0.5)
            
                self.move_cmd.linear.x = self.avoid_linear_speed
                self.move_cmd.angular.z = -copysign(self.avoid_angular_ratio, direction) / 300 * 0.4
                self.cmd_vel.publish(self.move_cmd)
                time.sleep(1.5)

    def set_patrol_mode(self):
            self.start_patrol = True

    def detect_whether_coming_in_qr_range(self):
        self.detect_qr = True
        while self.detect_qr == True:
            pass
        print("进入二维码识别范围")

    def trace_qr_central_point(self):
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
                    if ob['Area'] > self.qr_area_threshold[0] and ob['Area'] < self.qr_area_threshold[1]:
                        self.detect_qr = False
                        self.start_patrol = False # 关闭寻线，开始跟踪二维码中心点
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
                    self.move_cmd.linear.x = self.trace_liear_speed
                    self.move_cmd.angular.z = -self.trace_angular_ratio * temp / 150.0 * y / 224.0
                    self.cmd_vel.publish(self.move_cmd)
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
                    ob['CentralPoint'].append(msg.targets[i].rois[0].rect.x_offset + msg.targets[i].rois[0].rect.width/2)
                    ob['CentralPoint'].append(msg.targets[i].rois[0].rect.y_offset + msg.targets[i].rois[0].rect.height/2)
                    ob['Area'] = msg.targets[i].rois[0].rect.height * msg.targets[i].rois[0].rect.width
                elif msg.targets[i].type == "parking_sign\r":
                    ob['Type'] = "parking_sign"
                    ob['CentralPoint'].append(msg.targets[i].rois[0].rect.x_offset + msg.targets[i].rois[0].rect.width/2)
                    ob['CentralPoint'].append(msg.targets[i].rois[0].rect.y_offset + msg.targets[i].rois[0].rect.height/2)
                    ob['Area'] = msg.targets[i].rois[0].rect.height * msg.targets[i].rois[0].rect.width
                elif msg.targets[i].type == "obstacle1\r":
                    ob['Type'] = "obstacle"
                    ob['CentralPoint'].append(msg.targets[i].rois[0].rect.x_offset + msg.targets[i].rois[0].rect.width/2)
                    ob['CentralPoint'].append(msg.targets[i].rois[0].rect.y_offset + msg.targets[i].rois[0].rect.height/2)
                    ob['Area'] = msg.targets[i].rois[0].rect.height * msg.targets[i].rois[0].rect.width

                # 得到原始数据
                obs_lists.append(copy.deepcopy(ob)) # 深拷贝
        #print(obs_lists)
        self.obs_lists.clear()
        self.obs_lists = copy.deepcopy(obs_lists)
        #print(self.obs_lists)

    def command_callback_(self, msg):
        if 5 == msg.data:
            self.scan_qr_success = True
            print("二维码成功识别")

    def image_callback_(self, msg):
        num = 40

        np_img = np.frombuffer(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
        if 0 != len(self.line_central_point):
            x = self.line_central_point[0]
            y = self.line_central_point[1]
            cv2.circle(cv_img, (int(x), int(y)+256), 5, (0, 255, 0), -1)
        _, compressed_img = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, num])
        compressed_msg = CompressedImage()
        compressed_msg.format = 'jpeg'
        compressed_msg.data = compressed_img.tobytes()
        self.compressed_image_publisher_.publish(compressed_msg)

    def point_callback_(self, msg):
        central_point = []
        if 0 != len(msg.targets):
            for i in range(len(msg.targets)):
                central_point.append(msg.targets[i].points[0].point[0].x)
                central_point.append(msg.targets[i].points[0].point[0].y-256.0)
        self.line_central_point.clear()
        self.line_central_point = central_point
        print(self.line_central_point)

    def spin_task_(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = sunrise_controller("sunrise_controller")  # 新建一个节点
    time.sleep(8.0)
    #node.set_patrol_mode()
    # node.detect_whether_coming_in_qr_range()
    print("面积阈值已到")
    # node.trace_qr_central_point()
    #node.go_to_destination()
    print("已停车")

    while 1:
        pass
    # rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy


if __name__ == '__main__':
    main()
