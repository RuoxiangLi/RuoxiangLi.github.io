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

ORB-SLAM2系统追踪、局部建图、回环检测、可视化四个线程，其中追踪模块是在主线程中完成的。先介绍追踪模块的算法内容，这里从已经完成模块初始化开始。ORB_SLAM2中，重定位和闭环检测过程主要使用DBoW2来完成。

## Tracking部分代码分析

程序分为两种模式：**SLAM模式**和**Localization模式**，由变量`mbOnlyTracking`标记。SLAM模式中，三个线程全部都在工作，即在定位也在建图。而Localization模式中，只有Tracking线程在工作，即只定位，输出追踪结果（姿态），不会更新地图和关键帧。Localization模式主要用于已经有场景地图的情况下（在SLAM模式下完成建图后可以无缝切换到Localization模式）。Localization模式下追踪方法涉及到的关键函数是一样的，只是策略有所不同。

{% asset_img Track.png %}

###　初始追踪

追踪部分主要用了三种模型：运动模型（TrackWithMotionModel）、关键帧（TrackReferenceKeyFrame）和重定位（Relocalization）。三种跟踪模型都是为了获取相机位姿一个粗略的初值，后面会通过跟踪局部地图TrackLocalMap对位姿进行BundleAdjustment（捆集调整），进一步优化位姿。

1. TrackReferenceKeyFrame

   假设物体处于匀速运动，那么可以用上一帧的位姿和速度来估计当前帧的位姿。上一帧的速度可以通过前面几帧的位姿计算得到。这个模型适用于运动速度和方向比较一致、没有大转动的情形，比如匀速运动的汽车、机器人、人等。而对于运动比较随意的目标，当然就会失效了。此时就要用到下面两个模型。

2. TrackReferenceKeyFrame

   假如motion model已经失效，那么首先可以尝试和最近一个关键帧去做匹配。毕竟当前帧和上一个关键帧的距离还不是很远。作者利用了bag of words（BoW）来加速匹配。首先，计算当前帧的BoW，并设定初始位姿为上一帧的位姿；其次，根据位姿和BoW词典来寻找特征匹配（参见[ORB－SLAM（六）回环检测](http://www.cnblogs.com/luyb/p/5599042.html%20)）；最后，利用匹配的特征优化位姿（参见[ORB－SLAM（五）优化](http://www.cnblogs.com/luyb/p/5447497.html)）。

3. Relocalization

   假如当前帧与最近邻关键帧的匹配也失败了，意味着此时当前帧已经丢了，无法确定其真实位置。此时，只有去和所有关键帧匹配，看能否找到合适的位置。首先，计算当前帧的Bow向量。其次，利用BoW词典选取若干关键帧作为备选（参见[ORB－SLAM（六）回环检测](http://www.cnblogs.com/luyb/p/5599042.html%20)）；再次，寻找有足够多的特征点匹配的关键帧；最后，利用特征点匹配迭代求解位姿（RANSAC框架下，因为相对位姿可能比较大，局外点会比较多）。如果有关键帧有足够多的内点，那么选取该关键帧优化出的位姿。

### 位姿优化

姿态优化部分的主要思路是**在当前帧和（局部）地图之间寻找尽可能多的对应关系，来优化当前帧的位姿。**实际程序中，作者选取了非常多的关键帧和地图点。在跑Euroc数据集MH_01_easy时，几乎有一半以上的关键帧和地图点（后期>3000个）会在这一步被选中。然而，每一帧中只有200~300个地图点可以在当前帧上找到特征匹配点。这一步保证了非关键帧姿态估计的精度和鲁棒性。

### 局部地图更新

Tracking成功以后，需要更新motion model，并判断当前帧是否是新的关键帧。如果是，将其加入并更新局部地图（local 
map），建立当前关键帧与其它关键帧的连接关系，更新当前关键帧与其它关键帧之间的特征点匹配关系，*并利用三角法生成新的三维点（对每一个特征点，通过反投影得出3D地图点？）*，最后做一个局部优化（local BA，包括相邻关键帧和它们对应的三维点，参见[ORB－SLAM（五）优化](http://www.cnblogs.com/luyb/p/5447497.html)）。

## 参考资料

1. http://www.cnblogs.com/luyb/p/5357790.html
2. https://www.cnblogs.com/shang-slam/p/6395514.html