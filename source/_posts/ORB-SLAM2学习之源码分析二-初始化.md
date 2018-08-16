---
title: ORB_SLAM2学习之源码分析二-初始化
date: 2018-08-16 15:37:48
tags: 
  - ORB_SLAM2
categories: ORB_SLAM2
---

----

这篇文章是有关ORB_SLAM2系统源码分析的内容，主要记录单目、双目、RGB-D初始化过程，并进行比较。

<!--more--->

## 概述

SLAM过程初始化的目的是创建3d地图点，为后续跟踪提供初值。其中单目初始化较为复杂，双目、RGB-D初始化类似。

## 图像帧创建

在进行初始化之间都要将彩色图像处理成灰度图像（无论图片是RGB、BGR， 还是RGBA、BGRA，均转化为灰度图，放弃彩色信息），继而将图片封装成帧（`Frame`类对象）。下面介绍下图像帧的创建过程，三者初始化分别调用重载的`Frame`类构造函数。

单目创建图像帧函数调用：

~~~c++
mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBextractor, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
~~~

双目创建图像帧函数调用：

~~~c++
mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary,mK, mDistCoef, mbf, mThDepth);
~~~

RGB-D创建图像帧函数调用：

~~~c++
mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
~~~

ORB特征提取：

~~~c++
void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}
~~~

创建图像帧的关键一步是进行ORB特征提取，`ExtractORB()`调用了`ORBextractor`类中的重载操作符`void operator()`，完成特征提取，提取结果被保存在`Frame`类的成员变量`std::vector<cv:KeyPoint> mvKeys`和`cv:Mat mDescriptors`中。

## 单目初始化

单目初始化调用`Tracking::MonocularInitialization()`函数。

{% asset_img 单目初始化.png %}

### 重点分析

1. 需要获取连续两帧特征点数量超过100的图像帧，并且两帧图像匹配点大于100，才可以开始初始化，否则重新接收数据帧；连续两帧的前一帧设为参考帧；
2. 找到连续可用的图像帧后，需要使用专门的初始化器进行初始化（`Initializer.cc`），这个通过并行计算分解单应矩阵H和基础矩阵F，得到帧间运动（位姿）（关于并行计算F和H矩阵的初始化后续再做专门的记录）；
3. 创建初始地图过程。首先创建关键帧和地图点，通过将关键帧和地图点插入初始地图完成初始地图的构建，接着更新关键帧之间的连接关系（以共视地图点的数量作为权重），对两帧姿态图像进行全局优化重投影误差（和回环检测调整后的大回环优化使用的是同一函数）；
4. 比较重要的三个对象：地图、地图点、关键帧。地图就是整个的位姿和地图点。一个关键帧提取出的特征点对应一个地图点集，因此需要记下每个地图点在该帧中的编号；一个地图点会被多帧关键帧观测到，需要记下每个关键帧在该点中的编号。因此，地图点和关键帧的关系是：每个地图点会记录观测到自身的关键帧，关键帧中会记录观测到的所有地图点；
5. 创建地图点时，地图点中需要加入的一些属性：观测到该地图点的关键帧（以及对应的特征点）；该地图点的描述子（观测到该地图点的多个特征点中（对应多个关键帧），挑选出区分度最高的描述子，作为这个地图点的描述子）； 该MapPoint的平均观测方向和观测距离的，为后面做描述子融合做准备；
6. 局部地图、局部关键帧、局部地图点，是为了进行局部Bundle Adjustment。

### 单目初始化特点

1. 单目通过一帧无法估计深度，所以初始化时需要使用两帧图像；
2. 需要使用专门的初始化器进行初始化；
3. 为了让单目成功初始化（单目的初始化需要通过平移运动归一化尺度因子），初始化`Tracking`时`mpIniORBextractor`提取的特征点数量设定为普通帧的2倍（`Tracking.cc`）。

## 双目、RGB-D初始化

双目和RGB-D相机不需要通过两个相邻帧来恢复地图点深度，所以初始化过程极其相似，只要当前到来帧满足条件即可开始初始化，调用的是同一个函数`Tracking::StereoInitialization()`。

{% asset_img 双目初始化.png %}

### 重点分析

1. 创建初始地图过程。首先创建关键帧和地图点，通过将关键帧和地图点插入初始地图完成初始地图的构建。因为此初始化只用到一个图像帧，所以没有关键帧连接关系的更新和姿态的优化；
2. 其他同单目4、5、6条。

疑问：满足条件的第一个图像帧作为参考关键帧，后来帧都以该参考关键帧为参考吗？即参考关键帧会更新吗？

## 初始化比较

单目初始化的特点是双目、RGD-D初始化过程不具备的。（废话么这不是）

## 参考资料

1. https://www.cnblogs.com/shang-slam/p/6389129.html
2. https://www.cnblogs.com/mybrave/p/9342952.html