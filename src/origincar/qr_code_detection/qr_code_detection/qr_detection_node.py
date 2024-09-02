#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Int32
from origincar_msg.msg import Sign
from pyzbar.pyzbar import decode as pyzbar_decode
from pyzbar.pyzbar import ZBarSymbol
from std_msgs.msg import Int32
import time


class QRCodeReader(Node):
    def __init__(self):
        super().__init__('qrcode_reader')
        self.subscription = self.create_subscription(Image, 'image', self.image_callback, 10)
        self.publisher = self.create_publisher(Sign, 'sign_switch', 5)
        self.publisher1 = self.create_publisher(Int32, 'sign4return', 10)
        self.command_subscription = self.create_subscription(Int32, 'command', self.command_callback, 10)
        self.bridge = CvBridge()
        self.work_timer = self.create_timer(0.001, self.work_timer_callback_)
        self.start_time = False
        self.msg_data = 0
        

    def work_timer_callback_(self):
        if self.start_time == True:
            if self.msg_data == 2: # 顺时针，障碍物在后边
                print("self.msg_data", self.msg_data)
                time.sleep(3.75) # TODO
                sign_msg = Sign()
                sign_msg.sign_data = 3
                self.publisher.publish(sign_msg)
                
                msg1 = Int32()
                msg1.data = 5
                self.publisher1.publish(msg1)
                self.start_time = False
            elif self.msg_data == 3: # 逆时针，障碍物在后边
                print("self.msg_data", self.msg_data)
                time.sleep(3.75) # TODO
                sign_msg = Sign()
                sign_msg.sign_data = 4
                self.publisher.publish(sign_msg)
                
                msg1 = Int32()
                msg1.data = 5
                self.publisher1.publish(msg1)
                self.start_time = False
            elif self.msg_data == 4: # 顺时针，障碍物在前边
                print("self.msg_data", self.msg_data)
                time.sleep(7.0) # TODO
                sign_msg = Sign()
                sign_msg.sign_data = 3
                self.publisher.publish(sign_msg)
                
                msg1 = Int32()
                msg1.data = 5
                self.publisher1.publish(msg1)
                self.start_time = False
            elif self.msg_data == 5: # 逆时针，障碍物在前边
                print("self.msg_data", self.msg_data)
                time.sleep(7.0) # TODO
                sign_msg = Sign()
                sign_msg.sign_data = 4
                self.publisher.publish(sign_msg)
                
                msg1 = Int32()
                msg1.data = 5
                self.publisher1.publish(msg1)
                self.start_time = False

            

    def command_callback(self, msg):
        print("here")
        if msg.data == 1:
            pass
        else:
            self.start_time = True

        self.msg_data = msg.data


    def image_callback(self, msg):
    
        image_data = msg.data

    # 将图像数据转换为numpy数组
        image_np = np.frombuffer(image_data, dtype=np.uint8)

    # 使用cv2.imdecode()函数将numpy数组转换为OpenCV图像
        frame = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        decoded_objects = pyzbar_decode(gray, symbols=[ZBarSymbol.QRCODE])
        
        if decoded_objects:
            for obj in decoded_objects:
                codeinfo = obj.data.decode('utf-8')
                self.get_logger().info(f'QR Code detected: {codeinfo}')
                
                # Publish messages based on QR code content
                if codeinfo == 'AntiClockWise':
                    sign_msg = Sign()
                    sign_msg.sign_data = 4
                    self.publisher.publish(sign_msg)
                    
                    msg1 = Int32()
                    msg1.data = 5
                    self.publisher1.publish(msg1)
                    
                elif codeinfo == 'ClockWise':
                    sign_msg = Sign()
                    sign_msg.sign_data = 3
                    self.publisher.publish(sign_msg)
                    
                    msg1 = Int32()
                    msg1.data = 5
                    self.publisher1.publish(msg1)
        else:
            # self.get_logger().info('No QR code detected')
            pass

def main(args=None):
    rclpy.init(args=args)
    qr_code_reader = QRCodeReader()
    rclpy.spin(qr_code_reader)
    qr_code_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()