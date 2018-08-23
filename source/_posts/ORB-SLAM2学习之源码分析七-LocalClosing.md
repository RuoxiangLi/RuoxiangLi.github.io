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

LocalClosing模块是SLAM系统非常重要的一部分，由于VO过程存在累计误差（漂移），LocalClosing模块主要任务是检测闭环，即确定是否曾到达过此处，并在找到闭环之后计算Sim3变换，进行关键帧位姿、地图点的优化（后端优化），可以将这个累计误差缩小到一个可接受的范围内。闭环检测模块使用LocalMapping模块传送过来的关键帧，插入关键帧的操作是在LocalMapping线程主循环剔除冗余关键帧之后进行的，执行语句`mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame)`。

## 具体操作

{% asset_img LocalClosing.png %}

### 检测闭环

对应`LoopClosing::DetectLoop()`函数。检测回环候选（在`KeyFrameDataBase`中找到与`mlpLoopKeyFrameQueue`相似的闭环候选帧）并检查共视关系（检查候选帧连续性），函数会将所有通过一致性检测的候选关键帧统一放到集合`mvpEnoughConsistentCandidates`中，由下一步最终确定闭环帧。主要过程如下：

1. 计算当前帧`mpCurrentKF`和每一个与当前帧有共视关系的关键帧的Bow得分，得到最小得分**`minScore`** ；
2. 根据这个最小得分`minScore` 从`mpKeyFrameDB`（关键帧库）中找出候选的关键帧集合**`vpCandidateKFs`**；
3. 对于**`vpCandidateKFs`**里面的每一个关键帧，作为当前关键帧。找出其有共视关系的关键帧组成一个当前集合**`spCandidateGroup`。如果当前关键帧是`vpCandidateKFs`中第一帧的话，直接把这个`spCandidateGroup`集合，以分数0直接放到`mvConsistentGroups`中。**如果不是第一帧的话，就从**`mvConsistentGroups`**中依次取出里面的元素`pair<set<KeyFrame*>,int>`的`first`，这些元素也是关键帧们组成**以前集合`sPreviousGroup`**。只要是当前集合中的任意一个关键帧能在以前集合里面找到，就要将当前集合的得分加1，并把当前集合放到`mvConsistentGroups`里面**。**如果当前集合的得分大于3（`mnCovisibilityConsistencyTh`）的话**，当前帧就通过了一致性检测**，把当前帧放到**`mvpEnoughConsistentCandidates`，最后会找到很多候选的关键帧。下一步用sim3找出闭环帧。**注意该函数改变的就是**`mvpEnoughConsistentCandidates`**，`mvpEnoughConsistentCandidates`作为**该函数的输出**。

### 计算Sim3

对应`LoopClosing::ComputeSim3()`函数。该函数计算相似变换，从`mvpEnoughConsistentCandidates`中找出真正的闭环帧`mpMatchedKF`。主要过程如下：

1. 将当前帧与候选的关键帧进行Bow匹配，当匹配的数目超过20，就为该候选参考帧和当前帧构造一个`sim3Solver`，即相似求解器；
2. 用相似求解器求解出位姿`Scm`，通过相似变化求找出更多的匹配地图点，然后进行优化，优化的结果足够好的话就把候选的关键帧作为当前帧的闭环帧；
3. 得到`mg2oScw`。

### 闭环矫正

对应`LoopClosing::CorrectLoop()`函数，完成回环地图点融合和位姿图优化。具体来说，在回环检测到之后，对当前帧（回环帧）周围的帧的影响、回边的连接以及地图点做出的相应的改变。主要过程如下：

1. 通过上一步计算出来的当前帧的相似变换`mg2oScw`，对当前帧周围的每一个关键帧进行相似变化修正；
2. 这些关键帧有了修正好的相似变化，将这些关键帧里面的看到的地图点进行修正，并修正这些关键帧的位姿T；
3. 因为第二步修正了地图点，现在需要更新当前帧的地图点，如果需要对关键点进行替换则完成替换；当然也需要更新当前帧周围关键帧里的地图点；
4. 因为找到了闭环，还需要在covisibility里面增加闭环边（不止一条边），然后进行EssentialGraph优化。

## 参考资料

1. [ORB-SLAM2学习7 LocalClosing.h](https://www.cnblogs.com/panda1/p/7001042.html)
2. [ORB-SLAM（十）LoopClosing](https://www.cnblogs.com/shang-slam/p/6444037.html)
3. [ORBSlam2学习研究(Code analysis)-ORBSlam2中的闭环检测和后端优化LoopClosing](https://blog.csdn.net/chishuideyu/article/details/76165461)
4. [单目slam LoopClosing之Sim3优化](https://blog.csdn.net/stihy/article/details/62219842?locationNum=8&fps=1)