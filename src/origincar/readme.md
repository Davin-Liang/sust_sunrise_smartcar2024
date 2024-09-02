# 科大小虎队

## 一、操作步骤

1. 启动集成 launch

```
ros2 launch little_tigers_control sust_tigers.launch.py
```

2. 启动上位机

```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

3. 启动比赛控制节点

```
ros2 launch racing_control lingkage_patrol.launch.py
```

4. 按下启动键启动小车

```
python3 action.py
```

## 二、其余重要命令

1. 网卡 wifi 连接

```
sudo nmcli dev wifi connect "123" password "13467925" ifname wlx40a5ef24bb03
sudo nmcli dev wifi connect "134" password "13467925" ifname wlx40a5ef24bb03
```

2. 选择特定包进行编译

```
colcon build --packages-select racing_control
colcon build --packages-select little_tigers_control
```

3. 禁用板端 wifi

```
ifconfig wlan0 down
```

4. 固定 IP

```
nmcli connection modify 134 ipv4.method manual ipv4.addresses 192.168.11.111/24 ipv4.gateway 192.168.3.1 ipv4.dns 8.8.8.8 right
```

5. 扫描 wifi 网络

```
sudo nmcli device wifi rescan
```

6. 列出找到的 wifi

```
sudo nmcli device wifi list
```

# 三、重要子节点

1. 第一段巡线

```
ros2 launch racing_control first_patrol.launch.py
```

2. 第二段巡线

```
ros2 launch racing_control second_patrol.launch.py
```

3. 赛道检测

```
ros2 launch racing_track_detection_resnet racing_track_detection_resnet.launch.py
```

4. 底盘控制

```
ros2 launch origincar_base origincar_bringup.launch.py
```

5. 二维码检测

```
ros2 launch qr_code_detection qr_code_detection.launch.py
```

6. 启动相机及 YOLO 检测

```
ros2 launch dnn_node_example dnn_node_example.launch.py
```

7. 图像传输

```
ros2 run opencv_use my_opencv
```

# 四、快捷指令

1. 启动 sust_tigers.launch.py

```
launchs
```

2. 启动上位机

```
bridge
```

3. 启动比赛控制节点

```
racing
```

4. 查看比赛说明文档

```
tigers
```
