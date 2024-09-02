# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():

    prefix_path = os.path.expanduser("~/dev_ws/src/origincar/opencv_use") # 可能路径不对
    config_file_path = os.path.join(prefix_path, "config/line.yaml")
    print("config_file_path is ", config_file_path)

    declare_param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=TextSubstitution(text=str(config_file_path)),
        description='Full path to the ROS2 parameters file to use for the nodes'
    )

    param_file_path = LaunchConfiguration('param_file')

    return LaunchDescription([
        declare_param_file_arg,

        Node(
            package='opencv_use',
            executable='line_track',
            name='line_follower',
            output='screen',
            parameters=[param_file_path],
            arguments=['--ros-args', '--log-level', 'warn']
        )

    ])