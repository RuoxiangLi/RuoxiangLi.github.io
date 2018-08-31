---
title: ORB_SLAM2学习之源码分析九-ORB特征匹配
date: 2018-08-30 22:28:10
tags: 
  - ORB_SLAM2
categories: 
  - SLAM
  - ORB_SLAM2
---

----

这篇文章是有关ORB_SLAM2系统源码分析的内容，主要记录ORB特征匹配有关的内容。

<!--more---->

## 概述

ORB_SLAM2系统中，负责特征匹配的类是`ORBmatcher`，定义在ORBmatcher.h文件中。该类负责特征点与特征点之间，地图点与特征点之间通过投影关系、词袋模型或者Sim3位姿匹配，用来辅助完成单目初始化、三角化恢复新的地图点、Tracking、Relocalization以及Loop Closing等任务，因此比较重要。类中提供了多个接口，这些接口分别负责完成不同的匹配任务，下面进行一一介绍。

## 主要接口

### 重载的`SearchByProjection`函数

#### 用于追踪局部地图（Tracking）

对应函数：

~~~c++
int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3);
~~~

该函数负责在图像帧和投影的地图点之间搜索匹配，返回匹配到的点对数量。

#### 用于从上一图像帧追踪（Tracking）

对应函数：

~~~c++
int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);
~~~

该函数负责将从上一图像帧追踪到的地图点投影到当前帧，并搜索匹配，返回匹配到的点对数量，用于`TrackWithMotionModel`。

#### 用于重定位（Tracking）

对应函数：

~~~c++
int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);
~~~

该函数负责将从关键帧中观测到的地图点投影到当前帧，并搜索匹配（投影匹配当前帧的特征点），返回匹配到的点对数量，用于`Relocalization`。

#### 用于回环检测（LoopClosing）

对应函数：

~~~c++
int SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th);
~~~

该函数负责使用相似变换投影地图点，并搜索匹配，返回匹配到的点对数量，用于`Loop Closing`。

### 重载的`SearchByBow`函数

#### 用于重定位（Tracking）

对应函数：

~~~c++
int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);
~~~

该函数负责在当前帧中匹配关键帧中的地图点，用于`TrackReferenceKeyFrame`和`Relocalization`。

#### 用于回环检测（LoopClosing）

该函数负责在当前关键帧中匹配所有关键帧中的地图点，用于Loop Closing。

### 重载的`Fuse`函数

主要用于地图点的融合。如果地图点能匹配上当前关键帧的地图点，也就是地图点重合了，选择观测数多的地图点替换；地图点能匹配上当前帧的特征点，但是该特征点还没有生成地图点，则生成新的地图点。重载的第一个函数是为了减小尺度漂移的影响，需要知道当前关键帧的sim3位姿。

#### 使用相似变换的投影

对应函数：

~~~c++
int Fuse(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint);
~~~

该函数用于回环检测线程，负责使用一个给定的相似变换`Scw`将地图点投影到关键帧，搜索重叠的地图点。

#### 不使用相似变换的投影

对应函数：

~~~c++
int Fuse(KeyFrame* pKF, const vector<MapPoint *> &vpMapPoints, const float th=3.0);
~~~

该函数用于局部建图线程，负责将地图点投影到关键帧，搜索重叠的地图点。

### `SearchForInitialization`函数

函数原型：

~~~c++
int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);
~~~

该函数用于单目初始化过程，地图初始化时两个图像帧之间的匹配，只用于单目。

### `SearchForTriangulation`函数

函数原型：

~~~c++
int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2, cv::Mat F12,
              std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo);
~~~

该函数利用三角化，在两个关键帧之间恢复出一些地图点，检测对极约束。

### `SearchBySim3`函数

函数原型：

~~~c++
int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);
~~~

该函数在关键帧KF1和通过相似变换得到的关键帧KF2观测到的地图点之间搜索匹配，只用于双目和RGB-D。

### `DescriptorDistance`函数

对应函数：

~~~c++
static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);
~~~

该函数负责计算两个ORB描述子的汉明距离。

## 参考资料

1. [ORB-SLAM（八）ORBmatcher 特征匹配](https://www.cnblogs.com/shang-slam/p/6431017.html)