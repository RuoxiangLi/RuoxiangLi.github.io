---
title: Cartographer学习二源码分析之核心代码
date: 2018-09-16 13:52:13
tags: 
  - Lidar SLAM
  - Cartographer
mathjax: true
categories:
  - 机器人 
  - SLAM
  - Cartographer
copyright: true
---

----

本篇文章是记录激光开源SLAM系统Cartographer源码学习有关的内容。现在做的项目是基于师兄的毕业设计，使用的cartographer版本是20170320，和最新的系统已经有了挺大的差别，但大部分网上资料介绍的系统源码版本都比较旧，所以还是先学习旧版本的系统，搞明白核心内容后再了解更新的功能。

<!--more--->

## 概述

Google开源的代码包含两个部分：cartographer[1]和cartographer_ros[2]。cartographer主要负责处理来自雷达、IMU和里程计的数据并基于这些数据进行地图的构建，是cartographer理论的底层实现。cartographer_ros则基于ros的通信机制获取传感器的数据并将它们转换成cartographer中定义的格式传递给cartographer处理，与此同时也将cartographer的处理结果发布用于显示或保存，是基于cartographer的上层应用。ROS应用放在下篇文章分析。

先借用[知乎问题中mfxox](https://www.zhihu.com/question/51348391)的图：

{% asset_img 整体流程.png %}

## Cartographer代码框架

如下图所示，是google官方说明文档中的框架图：

{% asset_img Cartographer系统框架图.png %}

Cartographer系统可以分为两个互相有联系的独立系统：local SLAM（frontend）和global SLAM（backend）。

- 前端检测：local SLAM负责构建局部一致的submaps集合并进行融合，这个过程会产生漂移误差。

- 后端闭环优化：global SLAM运行在后端线程，负责在scans和submaps之间使用scan-matching找到回环约束，同时融合其他传感器数据（包括检测重力的方向），完成全局一致性任务。

  > 关于GraphSLAM原理，可以[参考文章](https://blog.csdn.net/heyijia0327/article/details/47686523)，有例子介绍很清晰。

总得来说，前端负责生成高质量的submaps，后端则负责完成地图的全局一致性任务。

输入的传感器数据有四个：

- Range Data（激光雷达，摄像头等）
- Odometry Pose(里程计数据)
- IMU Data
- FixedFramePose（是指确定的位置？？）

里程计数据与IMU数据共同进入PoseExtraPolator，做航迹推算。给定一个里程计与IMU得到的位置估计，用两次以上里程计计算平均速度，用两次以上IMU数据计算平均角速度，然后推算出下一时刻的位置姿态，给到Scan Matching中作为扫描匹配的初值。

Range Data数据经过体素滤波（一种滤波方法）和自适应体素滤波，进入scanMatching，作为观测值。scanMatching用论文中介绍的基于ceres优化的scanMatching方法获得观测最优位置估计，经过Motion Filter滤波，作为位置最优估计构建submap。

## Cartographer代码结构

### 各模块功能

- common：定义基本数据结构以及一些工具的使用接口，包括对数据的处理、时间转换、采样器、脉冲率计算、直方图类、数据计算类、对线程和线程锁的封装等
- io：文件流写、点云batch类、
- sensor：定义雷达数据及点云等相关的数据结构
- transform：定义位姿的数据结构及其相关的转换
- mapping：定义上层应用的调用接口以及局部submap构建和基于闭环检测的位姿优化等的接口
- kalman_filter：（基于UKF的多传感器数据融合）主要通过kalman滤波器完成对IMU、里程计及基于雷达数据的估计位姿的融合，进而估计新进的laser  scan的位姿
- mapping：定义上层应用的调用接口以及局部submap构建和基于闭环检测的位姿优化等的接口
- mapping_2d和mapping_3d：对mapping接口的不同实现（scan match策略对应的文件在这俩目录下的scan_matching目录中）

### mapping_2d

#### GlobalTrajectoryBuilder

实现接收处理上层应用传递的传感器数据的主要接口。有几个重要的成员函数：

- `AddRangefinderData`：用于接收处理上层应用传递的雷达数据
- `AddImuData`：用于接收处理上层应用传递的IMU数据
- `AddOdometerPose`：用于接收处理上层应用传递的里程计数据

#### LocalTrajectoryBuilder

local SLAM，主要完成完成UKF（扩展卡尔曼滤波器）、scan matching、局部submap的构建，提供了接收处理传感器数据的public函数：

- `AddImuData`：用于处理IMU数据

- `AddOdometerPose`：用于处理里程计数据

- `AddHorizontalLaserFan`：用于处理雷达数据

#### SparsePoseGraph

主要完成基于闭环检测的全局位姿优化。

## 参考资料

1. [cartographer文档](https://google-cartographer.readthedocs.io/en/latest/index.html#)
2. [cartographer ROS文档](https://google-cartographer-ros.readthedocs.io/en/latest/#)
3. [cartographer源码阅读（1）——算法整体结构](https://blog.csdn.net/u013721521/article/details/81477005)
4. [【SLAM】（一）Google Cartographer的初步尝试](https://blog.csdn.net/jsgaobiao/article/details/53116042)
5. [Cartographer理论及实现浅析](https://blog.csdn.net/u012700322/article/details/53513527)
6. **[Cartographer 代码阅读分析](https://blog.csdn.net/roadseek_zw/article/details/66968762)**
7. **[Cartographer 代码阅读分析-2](https://blog.csdn.net/roadseek_zw/article/details/72886079)**
8. [Cartographer源码分析58系列](https://blog.csdn.net/learnmoreonce/article/category/6989560/3)