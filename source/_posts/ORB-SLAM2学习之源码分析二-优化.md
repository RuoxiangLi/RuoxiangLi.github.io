---
title: ORB_SLAM2学习之源码分析二-优化
date: 2018-08-18 17:19:59
tags: 
  - ORB_SLAM2
categories: ORB_SLAM2
---

----

这篇文章是有关ORB_SLAM2系统源码分析的内容，主要记录单目、双目、RGB-D初始化过程，并进行比较。

<!--more--->

图优化

[知乎](https://www.zhihu.com/question/42050992)

covisibility graph叫共视图，顶点是key frame，如果两个key frame有相同的map points，就把这两个顶点连接起来。边的权重就是它们共享的3D点的个数。

essential graph是共视图的子集，顶点是关键帧，但只连接某个key frame和它拥有最多的map points的key frame.





闭环检测->闭环处优化：通过Sim3优化（以使得其尺度一致？？）