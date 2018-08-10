---
title: 视觉SLAM十四讲阅读笔记二-SLAM的数学表述
date: 2018-08-05 17:06:22
tags:
  - SLAM基础
  - 读书笔记
mathjax: true
categories: SLAM
---

-----

<!--more-->

ORB-SLAM2系统追踪、局部建图、回环检测、可视化四个线程，其中追踪模块是在主线程中完成的。



System.cc

TrackStereo、TrackRGBD、TrackMonocular分别输入左右视角图像、RGBD图像、图像以及时间戳，输出$Tcw$