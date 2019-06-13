---
title: Event-based特征追踪评估方法
date: 2019-06-13 13:55:04
tags:
  - Event-based Feature
  - Event Camera
categories: 
  - 机器人
  - SLAM
  - Event Camera
copyright: true
---
---

记录UZH机器感知实验室的一个Github项目学习过程，该项目目的是对Event-based特征追踪算法进行评估，包括轨迹的显示、误差、追踪时间的统计评测等。
<!--more--->

Github项目地址：https://github.com/uzh-rpg/rpg_feature_tracking_analysis

## 基于Event的特征追踪算法评估工作流程

特征追踪评估两个步骤：

- Ground Truth Tracker初始化
- Tracking

基于KLT追踪方法获取Ground Truth，算法执行步骤：

- 对于每个特征轨迹，确定其初始时间
- 在初始化时，或者初始化之后找到第一帧灰度图像
- 特征的x、y位置坐标插值到轨迹中，对应当前帧的时间戳
- KLT方法追踪该特征，直到该特征丢失；对于每一帧图像都要更新模板

基于Ground Truth的重投影，算法执行步骤：

- 对于每个特征轨迹，确定初始时间和位置
- 使用插值方法，确定每次初始化时的深度和位姿
- 使用位姿、深度、相机标定参数，back-projected特征
- 对于之后的每个位姿，将路标点重投影至图像帧中，生成特征轨迹
- 超出图像帧平面范围的特征将被丢弃

基于追踪的重投影过程不支持相机的畸变。

## 数据输入

需要使用`.txt`文件提供特征轨迹数据，内容包括特征位置坐标、特征ID、特征时间戳。

用于Ground Truth生成的图像、位姿、深度图、相机参数必须以rosbag的格式提供。各数据的信息格式如下：

- images: `sensor_msgs/Image`
- poses: `geometry_msgs/PoseStamped`
- depth maps: `sensor_msgs/Image`-`CV_32F`

有关ros话题和rosbag文件信息，使用`dataset.yaml`文件提供。

```
type: bag
name: relative/path/to/ros.bag  # relative path of bag

# For KLT based tracking 
image_topic: /dvs/image_raw  

# For reprojection based tracking
depth_map_topic: /dvs/depthmap
pose_topic: /dvs/pose
camera_info_topic: /dvs/camera_info
```

