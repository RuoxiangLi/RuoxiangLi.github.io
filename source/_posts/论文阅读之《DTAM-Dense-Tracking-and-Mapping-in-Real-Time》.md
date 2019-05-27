---
title: 论文阅读之《DTAM Dense Tracking and Mapping in Real-Time》
date: 2019-05-24 09:28:19
tags: 
  - SLAM
  - PTAM
  - 直接法SLAM
categories: 
  - 机器人
  - SLAM
  - 论文阅读
mathjax: true
copyright: true
---
---

终于有时间读论文了，DTAM——视觉SLAM直接法鼻祖。
<!--more--->

DTAM基于稠密的像素匹配方法，而不是特征提取方法，能够进行实时相机追踪和建图。

静态场景下移动RGB相机，系统估计详细的纹理深度地图。

基于关键帧的机制

基于非凸优化框架，最小化全局空间正则化能量函数

基于针对整个稠密模型的帧速率全图像对齐，精确追踪相机的6自由度位姿

使用GPU

DTAM (Dense tracking andmapping) 的目标函数中包含了多种数据关联的误差。包括图像空间的匹配误差(2D-2D)和3D 空间的位置误差(3D-3D)。当帧间运动较小，成功匹配的3D点较多时，使用3D-3D匹配估计位姿矩阵；当帧间运动较大，匹配2D点较多时，使用2d-2d匹配估计基础矩阵。

DTAM的direct method在默认环境亮度不变（brightness consistancy assumption）的前提下，  对每一个像素的深度数据进行inverse depth的提取和不断优化来建立稠密地图并实现稳定的位置跟踪。

## Mapping

> 构建稠密的3D表面模型

本模块的分析详见参考资料2。

构建**代价体素（Cost Volume）**。

## Tracking

> 基于3D模型进行稠密的相机位姿追踪，即全局的图像配准

## 参考资料

1. https://github.com/Ewenwan/MVision/tree/master/vSLAM/DTAM
2. 墙裂推荐：https://zhuanlan.zhihu.com/p/42137963
3. slides：https://wenku.baidu.com/view/3774d70c326c1eb91a37f111f18583d049640f04.html
4. 关于帧间相机旋转估计：http://campar.in.tum.de/twiki/pub/ISMAR08IAR/WebHome/pres.pdf