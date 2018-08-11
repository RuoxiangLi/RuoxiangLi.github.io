---
title: ROS学习之基于arbotix和rviz进行简单的机器人仿真
date: 2018-03-27 18:36:25
tags:
  - ROS
  - arbotix
  - rviz
categories: ROS
---

---

这篇文章是有关ROS中基于arbotix和rviz进行简单的机器人仿真的内容。

<!--more-->

在学习古月居大神关于ROS的博客时，参考他安装模拟器arbotix的方式总是连接超时，找到了如下源码安装方式。

### 本人的环境

ubuntu16.04+ROS(kinetic)

### 首先安装rbx1

rbx1是 *ROS by Example* 一书中的实例代码，该书是国外关于ROS出版的第一本书，主要针对Electric和Fuerte版本，使用机器人主要是TurtleBot。书中详细讲解了关于机器人的基本仿真、导航、路径规划、图像处理、语音识别等等，相关的代码基本都包含在`rbx１`中。rbx1源码安装方式：

~~~shell
cd ~/catkin_ws/src
git clone https://github.com/pirobot/rbx1.git 
cd ..
catkin_make
source ./devel/setup.bash
rospack profile
~~~

### 安装arbotix模拟器

~~~shell
cd ~/catkin_ws/src
git clone  https://github.com/vanadiumlabs/arbotix_ros.git
// 在arbotix_ros文件夹下新建文件夹src,将arbotix_ros目录下的所有文件剪切放到src文件夹下;
cd ..
catkin_make
~~~

后续测试模拟器等操作可以参考：

https://blog.csdn.net/lizilpl/article/details/46757683

和http://www.guyuehome.com/237

## rviz说明

显示的类型：

- Axes 显示坐标轴
- Effort 显示一个物体的边缘化 
- camera 提供一个窗口显示图像 
- grid 显示2D或者3D的一个栅格 
- Alpha 该参数表示透明度
- Ｌine Style 该参数表示线性（Billboards、Lines）
- grid cells 在一个网格中绘制八叉树地图
- iamge 用图像创造一个窗口 
- interactiveMake 允许用箭头来控制 
- Laser Scan 使用激光雷达进行扫描 
- Map 显示地图信息 
- Maker 允许程序员用topic 来控制 
- path 显示导航的路径 
- point 绘制出一些小的球 
- pose 绘制出位姿 
- ploygon 绘制多边形的轮廓线 
- Odometry 视觉里程计 
- range 显示视觉里程及声呐的测距范围 
- tf 显示tf变换的层次结构 
- RobotModel 显示机器人

### 动态修改参数

~~~c++
rosrun rqt_reconfigure rqt_reconfigure
~~~

该命令打开GUI中可以的动态调整ROS节点的参数，无需重新启动节点。