---
title: ORB_SLAM2学习之源码分析八-单目初始化再学习
date: 2018-08-29 21:48:50
tags: 
  - ORB_SLAM2
mathjax: true
categories: 
  - SLAM
  - ORB_SLAM2
---

---

这篇文章是有关ORB_SLAM2系统源码分析的内容，主要记录单目初始化过程中初始化器的工作，该过程通过对极几何方法计算基础矩阵、单应矩阵进而估计相机运动，再利用三角测量计算特征点的空间位置。

<!--more--->



