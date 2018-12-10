---
title: Cartographer学习三源码分析之ROS应用
date: 2018-09-19 22:02:01
tags: 
  - Lidar SLAM
  - Cartographer
  - rviz
mathjax: true
categories:
  - 机器人 
  - SLAM
  - Cartographer
copyright: true
---

---

本篇文章是记录cartographer_ros源码学习有关的内容，使用的cartographer版本是20170320。

<!--more--->

## 概述

cartographer_ros是基于cartographer的上层应用，它使用ROS作为通信机制，订阅各传感器数据节点发布到消息发布器的数据，然后打包封装成统一的数据格式，通过接口传入cartographer系统核心；通过接口获取系统核心处理后的数据，发布到相应的消息发布器，由rviz展示。下面是cartographer_ros系统运行时节点图：

{% asset_img cartographer_old.png %}

## ROS应用框架

下图是一个更简洁的框架图：

{% asset_img ROS框架.png %}

其中，cartographer_node是ROS应用的主要节点，它订阅了很多话题，其中/scan是激光雷达的数据，/imu是IMU的数据，又发布了很多数据用于展示。

## RVIZ配置文件

如果是无参数启动rviz，会默认使用配置`~/.rviz/default.rviz`。如果是正常退出，这一次的配置就会被保存到`~/.rviz/default.rviz`，需要另外保存成配置文件的话，可以选择`File->Save Config As`。

如果使用自己的配置文件，需要在`.launch`文件中添加内容：

~~~xml
<launch>
<node pkg="rviz" type="rviz" name="rviz"
    args="-d $(find cartographer_ros)/configuration_files/demo_3d.rviz"/>
</launch>
~~~

## 运行bag文件

如果需要使用`roslaunch`使用`rosbag`运行bag文件，在launch文件中添加内容：

~~~xml
<launch>
  <node pkg="rosbag" type="play" name="playe" output="screen" args="--clock /home/path/to/bagfile/2017-10-18-21-30-41.bag"/>
  <!-- 注意bag文件的路径必须为绝对路径-->
</launch>
~~~

bag文件的路径可以通过启动参数确定，使用`$(arg bag_filename)`替换上面的绝对路径。参数值可以通过命令行赋值，命令格式：

~~~shell
roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/slam/example_bags/b3-2016-04-05-14-14-00.bag
~~~

