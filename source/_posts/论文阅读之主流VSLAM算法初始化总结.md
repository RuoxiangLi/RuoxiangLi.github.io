---
title: 论文阅读之主流VSLAM算法初始化总结
date: 2019-05-23 12:17:29
tags:
categories:
mathjax:
copyright:
---
---

-
<!--more--->

MonoSLAM：初始化过程需要将相机放置在一个距离已知的平面场景下。

PTAM：初始化过程，计算**单应矩阵**，分解出旋转平移矩阵，作为相机初始位姿。

