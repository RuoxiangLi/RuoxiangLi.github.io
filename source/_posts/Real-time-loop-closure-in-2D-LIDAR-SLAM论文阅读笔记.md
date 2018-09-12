---
title: Real-time loop closure in 2D LIDAR SLAM论文阅读笔记
date: 2018-09-12 16:56:16
tags: 
  - 激光slam
mathjax: true
categories:
  - 机器人 
  - SLAM
  - 论文笔记
---

---

本篇文章记录阅读Google开源Cartographer SLAM系统论文过程中的学习内容。

<!--more-->

## 概况

`Real-time loop closure in 2D LIDAR SLAM`是Google发表在ICRA2016会议上的一篇论文，开源的系统是大名鼎鼎的Cartographer，目前该系统已经有大神改到Cartographer-ROS版本。本文在阅读论文的基础上，参考其他网络博客资料，学习并记录论文的一些要点，通过这个过程希望能够理解论文的核心内容和系统的实现。

## 文章贡献

文章的重点不是关注SLAM本身，而是提出了一种基于激光的5cm分辨率实时建图和回环检测方法，减少了计算量，满足实时的大场景地图构建以及大规模的实时优化的性能需求。

为了达到实时闭环检测，文章使用了分支上界法来计算scan-to-submap的匹配作为约束。

## Scan Matching方法介绍

涉及到的相关文献在文后列出，以便以后学习。

1. scan-to-scan matching：基于激光的SLAM中最常用来估计相关位姿的方法。但是非常容易造成累积误差。[1,2,3,4]

2. scan-to-map matching：可以较少累计误差。其中一种方法是使用Gauss-Newton法在线性插值地图上找到局部最优值，前提是获取到了最优的初始位姿估计，这就需要使用有足够高数据获取速率的激光雷达，以保证。局部优化的scan-to-map匹配是高效并且鲁棒的。在不稳定的平台上，使用惯性测量单元（IMU）将激光扫描投影到水平面上以估计重力方向。[5]

3. pixel-accurate scan matching：可以进一步减少局部误差，但是计算量比较大。这个方法同样可以用来检测回环。[1]

4. 从laser scans中提取特征做匹配，从而减少计算量[4]。

5. histogram-based matching用于回环检测[6]。

6. 用机器学习做laser scan data的特征检测[7]。

## 累积误差处理方式

1. 基于粒子滤波的优化。粒子滤波在处理大场景地图时，由于粒子数的极具增长造成资源密集。[8,9,10]

2. 基于位姿图的优化。与视觉SLAM的位姿图优化大同小异，主要是在观测方程上的区别。[2,11,12,13]

## 系统概述

Cartographer是实时的室内建图系统，能够生成5cm分辨率的2D网格地图。

laser scans数据被插入到submap中最佳估计位置，并假定最佳估计位置在短时间内足够准确。

scan matching针对最近的submap发生，因此它只和最近的scans和全局帧位姿估计中累计的误差有关系。

使用pose optimization处理误差累计。

一旦生成submap就不会再插入新的扫描。

回环检测过程的scan matching用到submap，并且会将所有已经生成的submaps和scans考虑在内。

如果有scan和submap在距离上足够的近，则scan matcher会尝试在submap中寻找回环scan matching。当一个新的laser scan加入到地图中时，如果该laser scan的估计位姿与地图中某个submap的某个laser scan的位姿比较接近的话，那么通过某种 scan match策略就会找到该闭环。

为了减少计算量，Cartographer设置了特殊的策略来找到回环scan matching。这个策略就是根据当前的位姿估计附近设置搜索窗口，在这个搜索窗口内执行branch-and-bound方法来寻找回环scan matching，如果在搜索窗口中找到了一个足够好的match，则会将该匹配添加到优化问题的回环检测约束条件中。

系统通过使用branch-and-bound方法并对每个生成的submap预计算出几个网格，保证回环优化的快速完成，快到可以在加入新的submap之前就完成优化，从而保证了一种软实时约束。

## Local 2D SLAM





## 参考资料

1. Real-time loop closure in 2D LIDAR SLAM
2. [google cartographer的论文《real-time loop closure in 2D LIDAR SLAM》翻译](https://blog.csdn.net/lilynothing/article/details/60875825)
3. [Real-Time Loop Closure in 2D LIDAR SLAM 论文笔记](https://zhehangt.github.io/2017/05/01/SLAM/CartographerPaper/)

## 相关文献

[1] E. Olson, “M3RSM: Many-to-many multi-resolution scan matching,” in Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), June 2015.
[2] K. Konolige, G. Grisetti, R. Kümmerle, W. Burgard, B. Limketkai, and R. Vincent, “Sparse pose 
adjustment for 2D mapping,” in IROS, Taipei, Taiwan, 10/2010 2010.
[3] F. Lu and E. Milios, “Globally consistent range scan alignment for environment mapping,” Autonomous robots, vol. 4, no. 4, pp. 333–349, 1997.
[4] F. Martı́n, R. Triebel, L. Moreno, and R. Siegwart, “Two different tools for three-dimensional mapping: DE-based scan matching and feature-based loop detection,” Robotica, vol. 32, no. 01, pp. 19–41,2014.
[5] S. Kohlbrecher, J. Meyer, O. von Stryk, and U. Klingauf, “A flexible and scalable SLAM system with full 3D motion 
estimation,” in Proc. IEEE International Symposium on Safety, Security and Rescue Robotics (SSRR). IEEE, November 2011.
[6] M. Himstedt, J. Frost, S. Hellbach, H.-J. Böhme, and E. Maehle, “Large scale place recognition in 2D LIDAR scans using geometrical landmark relations,” in Intelligent Robots and Systems (IROS 2014),2014 IEEE/RSJ International 
Conference on. IEEE, 2014, pp. 5030–5035.
[7] K. Granström, T. B. Schön, J. I. Nieto, and F. T. Ramos, “Learning to close loops from range data,” The International Journal of Robotics Research, vol. 30, no. 14, pp. 1728–1754, 2011.
[8] G. Grisetti, C. Stachniss, and W. Burgard, “Improving grid-based SLAM with Rao-Blackwellized particle filters by adaptive proposals and selective resampling,” in Robotics and Automation, 2005. ICRA 2005. Proceedings of the 2005 IEEE International Conference on. IEEE, 2005, pp. 2432–2437.
[9] G. D. Tipaldi, M. Braun, and K. O. Arras, “FLIRT: Interest regions for 2D range data with applications to robot navigation,” in Experimental Robotics. Springer, 2014, pp. 695–710.
[10] J. Strom and E. Olson, “Occupancy grid rasterization in large environments for teams of robots,” in Intelligent
 Robots and Systems (IROS),2011 IEEE/RSJ International Conference on. IEEE, 2011, pp. 4271–4276.
[11] R. Kümmerle, G. Grisetti, H. Strasdat, K. Konolige, and W. Burgard,“g2o: A general framework for graph optimization,” in Robotics and Automation (ICRA), 2011 IEEE International Conference on. IEEE,2011, pp. 3607–3613.
[12] L. Carlone, R. Aragues, J. A. Castellanos, and B. Bona, “A fast and accurate approximation for planar pose graph optimization,” The International Journal of Robotics Research, pp. 965–987, 2014.
[13] M. Bosse and R. Zlot, “Map matching and data association for large-scale two-dimensional laser scan-based SLAM,” The International Journal of Robotics Research, vol. 27, no. 6, pp. 667–691, 2008.
[14] E. B. Olson, “Real-time correlative scan matching,” in Robotics and Automation, 2009. ICRA’09. IEEE International Conference on. IEEE, 2009, pp. 4387–4393.

