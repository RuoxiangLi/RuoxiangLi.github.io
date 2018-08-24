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

   > 使用`KeyFrameDatabase::DetectLoopCandidates`完成候选帧的筛选，具体过程：
   >
   > 1. Bow得分>minScore；
   > 2. 统计满足1的关键帧中有共同单词最多的单词数maxcommonwords；
   > 3. 筛选出共同单词数大于mincommons(=0.8*maxcommons)的关键帧；
   > 4. 相连的关键帧分为一组，计算组得分（总分），得到最大总分bestAccScore，筛选出总分大于minScoreToRetain(=0.75*bestAccScore)的组，用组中得分最高的候选帧lAccScoreAndMatch代表该组。计算组得分的目的是剔除单独一帧得分较高，但是没有共视关键帧，作为闭环来说不够鲁棒。

3. 对于**`vpCandidateKFs`**里面的每一个关键帧，作为当前关键帧。找出其有共视关系的关键帧组成一个当前集合**`spCandidateGroup`。如果当前关键帧是`vpCandidateKFs`中第一帧的话，直接把这个`spCandidateGroup`集合，以分数0直接放到`mvConsistentGroups`中。**如果不是第一帧的话，就从**`mvConsistentGroups`**中依次取出里面的元素`pair<set<KeyFrame*>,int>`的`first`，这些元素也是关键帧们组成**以前集合`sPreviousGroup`**。只要是当前集合中的任意一个关键帧能在以前集合里面找到，就要将当前集合的得分加1，并把当前集合放到`mvConsistentGroups`里面**。**如果当前集合的得分大于3（`mnCovisibilityConsistencyTh`）的话**，当前帧就通过了一致性检测**，把当前帧放到**`mvpEnoughConsistentCandidates`，最后会找到很多候选的关键帧。下一步用sim3找出闭环帧。**

   > 注意该函数改变的就是**`mvpEnoughConsistentCandidates`**，`mvpEnoughConsistentCandidates`作为**该函数的输出**。

### 计算Sim3

对应`LoopClosing::ComputeSim3()`函数。该函数计算相似变换，从`mvpEnoughConsistentCandidates`中找出真正的闭环帧`mpMatchedKF`。主要过程如下：

1. 通过`SearchByBow`，搜索当前关键帧中和候选帧匹配的地图点，当匹配的数目超过20，就为该候选帧和当前帧构造一个`sim3Solver`，即相似求解器；

2. 用相似求解器`Sim3Solver`求解出位姿`Scm`，这里使用RANSAC方法；

3. 根据计算出的位姿，通过相似变化求找出更多的匹配地图点（`SearchBySim3`），更新`vpMapPointMatches`；

4. 使用更新后的匹配，使用g2o**优化Sim3位姿**（`OptimizeSim3`），优化的结果足够好的话（内点数nInliers>20）就把候选的关键帧作为当前帧的闭环帧`mpMatchedKF`，break，跳过对其他候选闭环帧的判断，同时也得到了`mScw`；

5. 获取`mpMatchedKF`及其相连关键帧对应的地图点。调用`SearchByProjection`将这些地图点通过上面优化得到的`mScw`变换投影（重投影）到当前关键帧进行匹配，若匹配点>=40个，则返回ture，进行闭环调整；否则，返回false，线程暂停5ms后继续检查Tracking发送来的关键帧队列。

   > 注意`SearchByProjection`得到的当前关键帧中匹配上闭环关键帧共视地图点（**mvpCurrentMatchedPoints**），将用于后面CorrectLoop时当前关键帧地图点的冲突融合。

至此，完成第4、5操作后，不仅确保了当前关键帧与闭环帧之间匹配度高，而且保证了闭环帧的共视图中的地图点和当前关键帧的特征点匹配度更高（20--->40），说明该闭环帧是正确的。

### 闭环矫正

对应`LoopClosing::CorrectLoop()`函数，完成回环地图点融合和位姿图优化。具体来说，在回环检测到之后，对当前帧（回环帧）周围的帧的影响、回边的连接以及地图点做出的相应的改变。闭环矫正时，LocalMapper和Global BA必须停止。注意Global BA使用的是单独的线程。主要过程如下：

1. 使用计算出的当前帧的相似变换Sim3，即`mScw`，对当前位姿进行调整，并且传播到当前关键帧相连的关键帧（相连关键帧之间相对位姿是知道的，通过当前关键帧的Sim3计算相连关键帧的Sim3）；
2. 经过上一步处理，回环两侧的关键帧完成对齐，然后利用调整过的位姿更新这些相连关键帧对应的地图点（修正关键帧看到的地图点）；
3. 将闭环帧及其相连帧的地图点都投影到当前帧以及相连帧上，投影匹配上的和Sim3计算过的地图点进行融合（就是替换成质量高的）；
4. 涉及融合的关键帧还需要更新其在共视地图中的观测边关系，因为找到了闭环，需要在covisibility里面增加闭环边（不止一条边）。这是为了剥离出因为闭环产生的新的连接关系LoopConnections，用于优化Essential Graph。添加当前帧与闭环匹配帧之间的边，该边不参与优化；
5. 然后进行EssentialGraph优化（只是优化一些主要关键帧）；
6. 新开一个线程进行全局优化，优化所有的位姿与MapPoints。

## 参考资料

1. [ORB-SLAM2学习7 LocalClosing.h](https://www.cnblogs.com/panda1/p/7001042.html)
2. [ORB-SLAM（十）LoopClosing](https://www.cnblogs.com/shang-slam/p/6444037.html)
3. [ORB-SLAM（六）回环检测](https://www.cnblogs.com/luyb/p/5599042.html)
4. [ORBSlam2学习研究(Code analysis)-ORBSlam2中的闭环检测和后端优化LoopClosing](https://blog.csdn.net/chishuideyu/article/details/76165461)
5. [单目slam LoopClosing之Sim3优化](https://blog.csdn.net/stihy/article/details/62219842?locationNum=8&fps=1)