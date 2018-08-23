---
title: ORB_SLAM2学习之源码分析七-LocalClosing
date: 2018-08-23 12:38:31
tags: 
  - ORB_SLAM2
categories: 
  - SLAM
  - ORB_SLAM2
---

---

这篇文章是有关ORB_SLAM2系统源码分析的内容，主要记录LocalClosing模块部分。

<!--more--->

## 概述

LocalClosing模块是SLAM系统非常重要的一部分，由于VO过程存在累计误差（漂移），LocalClosing模块主要任务是检测闭环，即确定是否曾到达过此处，并在找到闭环之后计算Sim3变换，进行关键帧位姿、地图点的优化（后端优化），可以将这个累计误差缩小到一个可接受的范围内。闭环检测模块使用LocalMapping模块传送过来的关键帧，插入关键帧的操作是在LocalMapping线程主循环剔除冗余关键帧之后进行的，执行`mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame)`。

## 具体操作

{% asset_img LocalClosing.png %}

### 检测闭环

对应`LoopClosing::DetectLoop()`函数。检测回环候选并检查共视关系。

### 计算Sim3

对应`LoopClosing::ComputeSim3()`函数。计算相似变换。

### 闭环矫正

对应`LoopClosing::CorrectLoop()`函数。执行回环融合和位姿图优化。



## 参考资料

1. [ORB-SLAM2学习7 LocalClosing.h](https://www.cnblogs.com/panda1/p/7001042.html)
2. [ORBSlam2学习研究(Code analysis)-ORBSlam2中的闭环检测和后端优化LoopClosing](https://blog.csdn.net/chishuideyu/article/details/76165461)
3. [单目slam LoopClosing之Sim3优化](https://blog.csdn.net/stihy/article/details/62219842?locationNum=8&fps=1)