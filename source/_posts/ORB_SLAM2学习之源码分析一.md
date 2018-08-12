---
title: ORB_SLAM2学习之源码分析一
date: 2018-08-12 20:01:21
tags: 
  - ORB_SLAM2
categories: ORB_SLAM2
---

---

这篇文章是有关ORB_SLAM2系统源码分析的内容。

<!--more-->

ORB-SLAM2系统追踪、局部建图、回环检测、可视化四个线程，其中追踪模块是在主线程中完成的。



System.cc

TrackStereo、TrackRGBD、TrackMonocular分别输入左右视角图像、RGBD图像、图像以及时间戳，输出