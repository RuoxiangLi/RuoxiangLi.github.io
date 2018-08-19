---
title: ROS学习之工作空间、软件包的创建和编译
date: 2018-05-10 21:42:22
tags:
  - ROS 
categories: 
  - 机器人
  - ROS
---

-----

这篇文章是有关ROS中工作空间、软件包创建以及编译的内容。

<!--more--->

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

**提示：**如果工作空间下包很多，每次都使用catkin_make的话效率十分低下，因为这种编译方法会编译工作空间下的所有的包，特别是在调试程序过程中会经常修改CMakeLists.txt文件里的内容，这样每次修改都要编译整个工作空间。所以可以使用ROS的catkin_make的功能编译一个或者多个包，具体的命令是：

`catkin_make  -DCATKIN_WHITELIST_PACKAGES="package_name")`

例如：`catkin_make  -DCATKIN_WHITELIST_PACKAGES="ros_slam"`

如果需要编译两个或者多个只需要中间加分号即可：

`catkin_make -DCATKIN_WHITELIST_PACKAGES="ros_slam;cv_bridge"`