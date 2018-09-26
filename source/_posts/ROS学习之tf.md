---
title: ROS学习之tf
date: 2018-04-04 15:05:37
tags:
  - ROS
categories: 
  - 机器人
  - ROS
---

-----

这篇文章是ROS tf有关的学习内容。

<!--more-->

## 概述

一个机器人系统有很多三维的坐标系，随时都在发生变化。tf可以对所有的坐标系进行实时监控，以供查询坐标系以及坐标系与坐标系之间的关系和关系的变化情况。rf可以在分布式系统中操作，意味着一个机器人的所有的坐标信息都能提供给系统中的任何主机上面的所有ROS组件。

- tf没有转换信息的中央服务器。
- tf默认坐标系都是右手坐标系，x轴向前、y轴向左、z轴向上。

任何用户使用tf基本上都会有两个任务，即监听变换和广播变换。

1. 使用tf必须要监听变换

   监听变换：接收并缓存系统中广播的所有坐标系，在坐标系中查找特定变换。

2. 如果希望扩展机器人的能力，需要开启广播变换：

   广播变换：发送坐标系的相关位姿到系统的其他部分。一个系统就可以有很多广播者，每一个都会提供机器人不同部位的信息。

> translation：平移
>
> rotation：旋转
>
> Transform(ations)：变换

## 数据类型

|  **Type**  |      **tf**      |
| :--------: | :--------------: |
| Quaternion | `tf::Quaternion` |
|   Vector   |  `tf::Vector3`   |
|   Point    |   `tf::Point`    |
|    Pose    |    `tf::Pose`    |
| Transform  | `tf::Transform`  |

其中四元数`tf::Quaternion`可以通过固定轴的Roll, Pitch and Yaw(滚动，俯仰和偏转)构造。

## tf工具

可以使用tf工具查看相关的信息。

### view_frames

`rosrun tf view_frames`：查看正在广播的坐标系的关系图，会生成`frames.pdf`，`evince frames.pdf`查看图。

### rqt_tf_tree

`rosrun rqt_tf_tree rqt_tf_tree`or`rqt &`：查看ROS中正在广播的坐标系的树形图。

### tf_echo

`rosrun tf tf_echo [reference_frame][target_frame]`：查看ROS中广播的任意两个坐标系之间的变换关系。



## 参考资料

1. [wiki官方文档](http://wiki.ros.org/tf)
2. [wiki官方Tutorials](http://wiki.ros.org/tf/Tutorials)
3. [ROS与C++入门教程](https://www.ncnynl.com/archives/201702/1308.html)
4. [坐标系统-古月居](http://www.guyuehome.com/265)