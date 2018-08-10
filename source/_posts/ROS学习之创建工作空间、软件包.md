---
title: ROS学习之创建工作空间、软件包
date: 2018-05-10 21:42:22
tags:
  - ROS 
categories: ROS
---

-----

## 创建工作空间

~~~~~shell
mkdir -p work_space/src
cd work_space
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH	
#查看环境变量包含内容：/home/.../work_space/src:/opt/ros/kinetic/share:/opt/ros/kinetic/stacks
~~~~~

## 创建程序包

~~~~~shell
cd work_space/src
catkin_create_pkg my_package std_msgs rospy roscpp
~~~~~

## 编译程序包

~~~~shell
cd work_space
source /opt/ros/kinetic/setup.bash
catkin_make
catkin_make install #可选
~~~~

