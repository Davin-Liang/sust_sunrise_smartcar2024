English| [简体中文](./README_cn.md)

# Function Introduction

Identifying the middle point of the track in images and publishing messages based on deep learning methods, using the ResNet18 model.

# Instructions for Use

## Preparation

Have a real robot or robot simulation module with a motion chassis, camera, and RDK kit that can operate properly.

## Installing the Package

**1. Installing the Package**

After starting the robot, connect to the robot via SSH or VNC through the terminal, click the "One-click Deployment" button on this page's top right corner, copy and run the following command on the RDK system to complete the installation of the relevant Nodes.

```bash
sudo apt update
sudo apt install -y tros-racing-track-detection-resnet
```

**2. Running the Line Following Perception Function**

```shell
source /opt/tros/local_setup.bash

# Visualize the track center on the web (open ip:8000 in the browser after launching the function)
export WEB_SHOW=TRUE

# Simulation (use simulation model)
ros2 launch racing_track_detection_resnet racing_track_detection_resnet_simulation.launch.py

# Real-world scenario (use the model in a real-world setting)
ros2 launch racing_track_detection_resnet racing_track_detection_resnet.launch.py
```


# Principle Overview

The Horizon RDK obtains environmental data in front of the car through the camera, infers the coordinate values of the guidance line through a well-trained CNN model on image data, and publishes them.

# Interface Description

## Topics

### Pub Topics

| Name                          | Message Type                                                | Description                                            |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /racing_track_center_detection | ai_msgs::msg::PerceptionTargets                            | Publishes the image coordinates of the track center   |
### Subtopic
| Name                          | Message Type                                                     | Description                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /hbmem_img                     | hbm_img_msgs/msg/HbmMsg1080P                                    | Receive image messages published by the camera (640x480)                   |

## Parameters

| Parameter                | Type        | Description   |
| --------------------- | ----------- | -------------------------------------------------------------------------------------------------- |
| model_path       | string | Path to the model file used for inference, please configure the actual model path, default value is /opt/nodehub_model/race_detection/race_track_detection_simulation.bin |
| sub_img_topic       | string |  Topic name for receiving images, please configure according to the actual topic received, default value is /hbmem_img |

# Note
This package provides models that can be used in gazebo simulation environments and specific models that can be used in actual scenarios. If you collect your own dataset for training, please remember to replace them accordingly.