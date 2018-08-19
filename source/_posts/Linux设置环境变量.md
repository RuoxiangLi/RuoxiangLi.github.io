---
title: Linux设置环境变量
date: 2018-08-07 16:03:24
tags:
  - ubuntu
categories: 
  - 系统
  - ubuntu
---

---

这篇文章是有关Linux中环境变量设置的内容记录。

<!--more--->

`export PATH=$PATH:/home/..... `(路径目录)：此方法只在当前终端有效 	

使用`echo $PATH`命令查看环境变量的内容

测试命令:export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/slam/ORB_SLAM2/Examples/ROS

（将路径永久添加到PATH）：

echo "source ~/slam/ORB_SLAM2/Examples/ROS/ORB_SLAM2/build/devel/setup.sh" >> ~/.bashrc

1. export定义一个全局变量
2. 使用echo $ROS_PACKAGE_PATH查看变量内容



下面这些命令都是什么作用：

`export PATH=$PATH:/home/..... `，只在当前终端有效，source ~/.bashrc也不行，如何永久有效？？？

`export`之后编译程序包，就会产生setup.sh文件，再执行如下命令即可：

echo "source ~/slam/ORB_SLAM2/Examples/ROS/ORB_SLAM2/build/devel/setup.sh" >> ~/.bashrc

`source /devel/setup.bash`

`source /opt/ros/kinetic/setup.bash`

全局变量、局部变量的设置？

在～/.bashrc文件中source或者export，打开终端会一直可用。