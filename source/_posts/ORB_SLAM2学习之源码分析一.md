---
title: ORB_SLAM2学习之源码分析一-Tracking
date: 2018-08-12 20:01:21
tags: 
  - ORB_SLAM2
categories: ORB_SLAM2
---

---

这篇文章是有关ORB_SLAM2系统源码分析的内容。

<!--more-->

ORB-SLAM2系统追踪、局部建图、回环检测、可视化四个线程，其中追踪模块是在主线程中完成的。

Tracking部分代码分析

程序分为两种模式：**SLAM模式**和**Localization模式**，由变量`mbOnlyTracking`标记。SLAM模式中，三个线程全部都在工作，即在定位也在建图。而Localization模式中，只有Tracking线程在工作，即只定位，输出追踪结果（姿态），不会更新地图和关键帧。Localization模式主要用于已经有场景地图的情况下（在SLAM模式下完成建图后可以无缝切换到Localization模式）。Localization模式下追踪方法涉及到的关键函数是一样的，只是策略有所不同。



System.cc

TrackStereo、TrackRGBD、TrackMonocular分别输入左右视角图像、RGBD图像、图像以及时间戳，输出

ORB_SLAM2中，主要使用DBoW2用来做重定位和闭环检测 

对每一个特征点，通过反投影得出3D地图点