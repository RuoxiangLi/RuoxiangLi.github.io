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

### 当前终端有效

`export PATH=$PATH:/home/..... `(路径目录)：此方法只在当前终端有效 	

使用`echo $PATH`命令查看环境变量的内容。

> 命令`source /devel/setup.bash`将当前工作空间加入环境变量，也是只对当前终端有效。

### 永久有效

将路径永久添加到PATH，每次启动终端都会找到路径：

`echo "source ~/slam/ORB_SLAM2/Examples/ROS/ORB_SLAM2/build/devel/setup.sh" >> ~/.bashrc`