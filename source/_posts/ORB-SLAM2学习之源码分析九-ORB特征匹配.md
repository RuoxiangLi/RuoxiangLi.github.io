---
title: ORB_SLAM2学习之源码分析九-ORB特征匹配
date: 2018-08-30 22:28:10
tags: 
  - ORB_SLAM2
categories: 
  - 机器人
  - SLAM
  - ORB_SLAM2
copyright: true
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

该函数负责在当前帧特征点和投影的地图点之间搜索匹配，具体来说，就是将投影的地图点`vpMapPoints`（这些地图点是上一帧的局部关键帧对应的所有局部地图点）投影到当前图像帧并搜索匹配，返回匹配到的点对数。具体匹配过程：

1. 对于`vpMapPoints`中的每个地图点，搜索其在当前图像帧一个窗口内的特征点；
2. 计算该地图点的描述子与窗口范围内的每一个特征点描述子的汉明距离；
3. 找到汉明距离值最小bestDist和次小bestDist2的两个最佳匹配特征点；
4. 如果两个特征点所属的金字塔层不同并且最佳匹配点的汉明距离值bestDist大于mfNNratio与次最佳匹配点的距离值bestDist2的乘积，则认为最佳匹配点就是汉明距离最小的这个特征点；将该地图点设为当前特征点对应的地图点，匹配成功，匹配数+1，继续循环匹配；
5. 否则，继续循环匹配；
6. 继续循环匹配，直到`vpMapPoints`中的每个地图点都匹配完毕（或匹配到或匹配不到）。

#### 用于从上一图像帧追踪（Tracking）

对应函数：

~~~c++
int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);
~~~

该函数负责将从上一图像帧追踪到的地图点投影到当前帧，并搜索匹配，返回匹配到的点对数量，用于`TrackWithMotionModel`。具体过程：

1. 建立旋转直方图，用于检测旋转一致性；

2. 分别计算当前帧和前一帧的旋转矩阵和平移矩阵；

3. 通过相机模型，将前一帧的每一个地图点，投影得到在当前帧中的像素坐标；

4. 在一个窗口内搜寻特征点`GetFeaturesInArea`，窗口尺寸根据尺度（特征点所处金字塔层）进行变化。因为前面ORB是进行了高斯金字塔分层，金字塔越上层的尺寸越小，搜索窗口相应就小；

5. 找到地图点描述子与在窗口中的特征点描述子之间的距离的最佳值（最小值），如果距离小于设定值`TH_HIGH`，则认为是匹配点对，匹配数+1；

6. 在直方图中记录特征点角度变化（做旋转一致性检测有关的记录，上一帧关键点角度和当前帧关键点角度相减）；直方图形式如下所示，旋转角度除以直方图长度，四舍五入得到横坐标（横坐标范围是`0-HISTO_LENGTH`），每个横坐标值对应一个`vector`集合，记录所有对应具有当前角度的匹配点的序号，纵坐标是集合的大小。

   {% asset_img 直方图.png %}

7. 循环3，继续匹配；

8. 应用旋转一致性再一次筛选匹配点对（根据角度变化剔除不好的匹配），具体做法是从直方图中选出纵坐标值最大的前三个集合，剔除这三个集合以外的其他匹配点。

#### 用于重定位（Tracking）

对应函数：

~~~c++
int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);
~~~

该函数负责将从关键帧中观测到的地图点投影到当前帧，并搜索匹配（投影匹配当前帧的特征点），返回匹配到的点对数量，用于`Relocalization`。具体过程与上一个函数类似。

#### 用于回环检测（LoopClosing）

对应函数：

~~~c++
int SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th);
~~~

该函数负责使用相似变换投影地图点到当前关键帧，并搜索匹配，返回匹配到的点对数量，用于`Loop Closing`。

##### 参数

- pKF：检测闭环时的当前关键帧
- Scw：闭环帧（从候选帧选出的）与当前帧之间的相似变换`Scm`
- vpPoints：闭环帧及其相连关键帧对应的地图点
- vpMatched：已经找到的地图点（当前关键帧与所有候选关键帧SearchByBoW匹配到的地图点）

##### 具体过程

1. 获取相机标定参数，为之后的投影做准备；
2. 分解相似变换矩阵：先计算得到尺度因子s，然后计算世界坐标系到相机坐标系pKF的旋转矩阵和平移向量（都是去掉尺度因子的）；
3. 在已经找到的匹配点中，去除没有对应匹配点的地图点；
4. 遍历每个地图点，不包括坏的地图点和已经找到的地图点
   - 首先获取到地图点的3D世界坐标系，并通过第2步计算得到的矩阵转换到相机坐标系下，且在Pc下的深度必须为正；
   - 再投影到像素坐标系下，坐标值必须在image范围内；
   - 计算世界坐标系下，该地图点的深度，该深度必须在该点的尺度方差区域内，且观察视角必须小于60°（通过向量内积来判断观察角度是否小于60°）；
   - 根据地图点的深度和关键帧预测一个高斯金字塔层，根据该层计算一个半径，获得该半径范围内的特征点；
   - 找出半径范围内所有特征点的描述子与地图点描述子汉明距离最小的特征点，如果满足最小距离`bestDist<=TH_LOW`，则认定匹配成功。

### 重载的`SearchByBow`函数

#### 用于重定位（Tracking）

对应函数：

~~~c++
int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);
~~~

该函数通过BoW对关键帧和当前图像帧的特征点进行匹配，用于`TrackReferenceKeyFrame`和`Relocalization`。

#### 用于回环检测（LoopClosing）

对应函数：

~~~c++
int ORBmatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12)
~~~

该函数通过BoW对两个关键帧（当前关键帧`pKF1`和候选关键帧之一`pKF2`）的特征点进行匹配，用于Loop Closing时对两个关键帧间的特征点匹配，返回匹配到的点对数量，将匹配到的特征点对应的地图点记录在`vpMatches12`中作为输出。

##### 参数

- pKF1：当前关键帧
- pKF2：候选关键帧中的一个
- vpMatches12：匹配到的地图点

##### 具体实现

1. 对四叉树中属于相同节点的特征点才考虑匹配，首先对KF1中某个节点下的特征点，遍历KF2中对应节点的所有特征点，计算描述子距离并记录信息；
2. 根据距离和阈值关系添加匹配，记录角度变化；
3. 根据角度变化剔除误匹配。

### 重载的`Fuse`函数

主要用于地图点的融合。如果地图点能匹配上当前关键帧的地图点，也就是地图点重合了，选择观测数多的地图点替换；地图点能匹配上当前帧的特征点，但是该特征点还没有生成地图点，则生成新的地图点。重载的第一个函数是为了减小尺度漂移的影响，需要知道当前关键帧的sim3位姿。

#### 使用相似变换的投影

对应函数：

~~~c++
int Fuse(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint);
~~~

该函数用于回环检测线程，负责使用一个给定的相似变换`Scw`将地图点投影到关键帧，搜索重叠的地图点。具体实现：

1. 分解相似变换矩阵，得到旋转矩阵和平移向量等数据；
2. 对每一个地图点投影到相机系和图像系，检查深度、视角等信息；
3. 由距离预测尺度进而确定搜索半径，在指定区域内获取特征点；
4. 遍历这些特征点，计算其描述子和地图点描述子的汉明距离，计录最好距离和索引；
5. 如果索引对应的地图点不存在则新添加一个，存在的话则标记其将要被替换。

#### 不使用相似变换的投影

对应函数：

~~~c++
int Fuse(KeyFrame* pKF, const vector<MapPoint *> &vpMapPoints, const float th=3.0);
~~~

该函数用于局部建图线程，负责将地图点投影到关键帧，搜索重叠的地图点，返回融合的地图点数量。具体实现：

1. 遍历地图点，将其转换至相机坐标系检查深度，投影并检查是否在图像内，检查尺度和视角范围是否合适；
2. 根据深度预测尺度从而确定搜索半径，获得范围内的特征点；
3. 遍历这些点，计算它们和地图点投影后的坐标误差，误差太大的跳过，计算描述子距离，记录距离最小的最佳匹配点；
4. 对于最佳匹配，如果某个点已经有了地图点，则选择观测次数多的那个点，舍弃观测次数少的点，即两点融合；如果没有对应的地图点，则添加地图点，添加观测关系。

### `SearchForInitialization`函数

函数原型：

~~~c++
int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);
~~~

该函数用于单目初始化过程，地图初始化时两个图像帧之间的匹配，只用于单目。具体实现：

1. 构造直方图；
2. 遍历第一帧中的特征点，对于每一个特征点`kp1`；
3. 首先在第二帧中设置搜索范围（与特征点`kp1`在同一金字塔层级），获取范围内的特征点；
4. 遍历搜索范围内的点，对于每一个特征点，计算其描述子与`kp1`描述子的汉明距离，记录相关信息；
5. 根据最好距离和次好距离以及阈值的关系，记录匹配，记录特征点角度变化到直方图；
6. 根据角度变化剔除不好的匹配。

### `SearchForTriangulation`函数

函数原型：

~~~c++
int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2, cv::Mat F12,
              std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo);
~~~

该函数利用三角化，在两个关键帧之间恢复出一些地图点，检测对极约束。具体实现：

1. 对第一个关键帧中的特征点进行遍历，跳过已经有对应地图点的点，因为这个函数的目的是为了三角化进行的搜索匹配；
2. 对于第二个关键帧中的特征点，计算描述子之间的距离，检查特征点距离极线的距离，满足要求的记录信息；
3. 进行角度变化的统计和检查。

### `SearchBySim3`函数

函数原型：

~~~c++
int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);
~~~

该函数在关键帧pKF1和通过相似变换得到的关键帧pKF2观测到的地图点之间搜索匹配，主要是匹配之前漏匹配的点，只用于双目和RGB-D。

通过sim3变换，确定pKF1中的特征点在pKF2中的大致区域，同理，确定pKF2的特征点在pKF1中的大致区域。在该区域内通过描述子进行匹配捕获pKF1和pKF2之前漏匹配的特征点，更新vpMatches12（之前使用SearchByBow进行特征点匹配时会有漏匹配）。最后有一个检查的条件，某一对匹配点在pKF1至pKF2的匹配及pKF2至pKF1的匹配都存在时才认为是有效的匹配。

### `DescriptorDistance`函数

对应函数：

~~~c++
static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);
~~~

该函数负责计算两个ORB描述子的汉明距离。

## 总结

1. 各种特征匹配的基本思路都是根据距离预测尺度，进而根据尺度确定一个合理的搜索范围，之后对这个范围内的点进行匹配，从匹配结果中选出满足条件的最佳匹配。
2. 何时用投影匹配，何时用DBow2进行匹配？
   - 在Relocalization和LoopClosing中进行匹配的是在很多帧关键帧集合中匹配，属于Place Recognition，因此需要用DBow。
   - 而投影匹配适用于两帧之间，或者投影范围内（局部地图，前一个关键帧对应地图点）的MapPoints与当前帧之间。
3. 以上函数在检测特征点时，都会用到角度直方图用来剔除不满足两帧之间角度旋转的外点，也就是旋转一致性检测。具体实现：
   - 将关键帧与当前帧匹配关键点的`angle`相减，得到rot（`0≤rot＜360`），放入一个直方图中，对于每一对匹配点的角度差，均可以放入一个`bin`的范围内（`360/HISTO_LENGTH`）；
   - 统计直方图最高的三个`bin`保留，其他范围内的匹配点剔除。

## 参考资料

1. [ORB-SLAM（八）ORBmatcher 特征匹配](https://www.cnblogs.com/shang-slam/p/6431017.html)
2. [ORBSLAM2源码学习（6） ORBmatcher类](https://blog.csdn.net/u012936940/article/details/81396061)
3. [ORBmatacher](https://www.cnblogs.com/Fighting_lan/p/7816712.html)