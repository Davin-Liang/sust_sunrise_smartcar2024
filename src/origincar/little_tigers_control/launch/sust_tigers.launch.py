from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import TimerAction, OpaqueFunction
# from launch import EmitEvent
# from launch import Shutdown
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

# 1. 在 root 目录下运行该 launch
# 2. 在 root 目录下得有 /obstacle_config 目录
# 3. 修改在 racing_obstacle_detection_yolo.launch.py 中 /yolov5sconfig.json 的路径：config/yolov5sconfig.json -> obstacle_config/yolov5sconfig.json


def generate_launch_description():
    # 底盘控制节点
    origincar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('origincar_base'), 'launch'),
            '/origincar_bringup.launch.py'])
    )

    # 二维码识别节点
    qr_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('qr_code_detection')),
            '/qr_code_detection.launch.py'])
    )
    
    # 图像转换
    opencv_node = Node(
            package='opencv_use',
            executable='my_opencv',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        )
        
    #opencv巡线
    #opencv_line = Node(
        #    package='opencv_use',
       #     executable='line_track',
        #    output='screen',
          #  arguments=['--ros-args', '--log-level', 'warn']
       #  )
         
    opencv_line = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('racing_control'), 'launch'),
            '/line.launch.py'])
    )
        
    # 赛道检测节点
    racing_tracking_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('racing_track_detection_resnet'), 'launch'),
            '/racing_track_detection_resnet.launch.py'])
    )

    # 警戒线检测节点 
    yolo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('dnn_node_example'), 'launch'),
            '/dnn_node_example.launch.py']),
    )

    # 寻线控制节点

    # critical_nodes = [driver_node, base_node, lidar_node, vision_node]

    # event_handlers = [
    #     RegisterEventHandler(
    #         OnStateTransition(
    #             target_lifecycle_node=node,
    #             goal_state='inactive',
    #             entities=[
    #                 LogInfo(msg=f"A critical node [{node.executable}] has stopped. Shutting down..."),
    #                 EmitEvent(event=Shutdown())
    #             ]
    #         )
    #     )
    #     for node in critical_nodes
    # ]

    def launch_other_nodes(context):
        return [
            yolo_node,                      # Yolo 检测
            origincar_node,             # 底盘控制
            #qr_node,                    # 二维码检测
            #racing_control_node,        # 寻线控制
            opencv_node,                  #图像转换
            # *event_handlers,
            #opencv_line
            #opencv_line                #opencv巡线
        ]

    delayed_start = TimerAction(
        period=3.0,  # 延迟 3 秒启动其他节点
        actions=[OpaqueFunction(function=launch_other_nodes)]
    )

    return LaunchDescription([
        racing_tracking_node,       # 赛道检测

        delayed_start,                  # 延迟启动
    ])
