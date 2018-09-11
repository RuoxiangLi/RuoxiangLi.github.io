---
title: ORB_SLAM2学习之源码分析三-优化
date: 2018-08-18 17:19:59
tags: 
  - ORB_SLAM2
mathjax: true
categories: 
  - 机器人
  - SLAM
  - ORB_SLAM2
---

----

这篇文章是有关ORB_SLAM2系统源码分析的内容，主要记录位姿优化有关的内容。

<!--more--->

## 概述

因为摄像机标定（camera  calibration）和追踪（tracking）的精度不够。摄像机标定的误差会体现在重建中（比如三角法重建时），而追踪的误差则会体现在不同关键帧之间的位姿中和重建中（单目）。误差的不断累积会导致后面帧的位姿离实际位姿越来越远，最终会限制系统整体的精度。

### 关于摄像机标定

单目SLAM文献中一般假设摄像机标定的结果是准确的，并不考虑这个因素带来的误差（大概因为很多时候跑标准的数据集，认为摄像机标定的误差是相似的）。然而对于一个产品，不同类型的传感器对应的标定误差并不相同，甚至有可能差异很大。因此，如果要评估整个系统的精度，这方面的误差必须要考虑进去。

### 关于追踪

无论在单目、双目还是RGBD中，追踪得到的位姿都是有误差的。单目SLAM中，如果两帧之间有足够的对应点，那么既可以直接得到两帧之间的位姿（像初始化中那样），也可以通过求解一个优化问题得到（如solvePnP）。由于单目中尺度的不确定性，还会引入尺度的误差（尺度漂移scale-drift）。由于tracking得到的总是相对位姿，前面某一帧的误差会一直传递到后面去，导致tracking到最后位姿误差有可能非常大。为了提高tracking的精度，可以1.  在局部和全局优化位姿；2. 利用闭环检测（loop closure）来优化位姿。

## 优化方法

在SLAM问题中，优化的目标函数常见的几种约束条件为：

  1. 三维点到二维特征的映射关系（通过投影矩阵）；

  2. 位姿和位姿之间的变换关系（通过三维刚体变换）；

  3. 二维特征到二维特征的匹配关系（通过F矩阵）；

  4. 其它关系（比如单目中有相似变换关系）。

     > ### 关于相似变换Sim3
     >
     > 相似变换比欧式变换多了一个自由度，有7个自由度，它允许物体进行均匀缩放（体积比不变），其矩阵表示为：
     >
     > $T_s=Sim3=\left[ \begin{matrix}sR &t\\ 0^T &  1   \end{matrix} \right]$
     >
     > 式中旋转部分多了一个缩放因子$s$，表示我们在对向量旋转之后，可以在x，y，z三个坐标上进行均匀缩放。由于含有缩放，相似变换不再保持图形的面积不变。

如果我们能够知道其中的某些关系是准确的，那么可以在g2o中定义这样的关系及其对应的残差，通过不断迭代优化位姿来逐步减小残差和，从而达到优化位姿的目标。下面介绍ORB-SLAM2系统中主要的优化函数，它们是定义在`Optimizer.h`文件中的静态函数，可以直接通过类名访问，而不必创建类的对象。ORB中使用的这些优化函数是非常重要的，在视觉SLAM中有很强的通用性，自己实现的时候完全可以参考其实现方法。

### 局部优化

#### 对应函数

~~~c++
void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap)
~~~

函数中优化求解器初始化过程如下：

~~~c++
g2o::SparseOptimizer optimizer;
g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
optimizer.setAlgorithm(solver);
~~~

#### 具体实现

用于LocalMapping线程中，在剔除关键帧之前进行的局部地图优化。当新的关键帧加入到convisibility  graph时，作者在关键帧附近进行一次局部优化，如下图所示。Pos3是新加入的关键帧，其初始估计位姿已经得到。此时，Pos2是和Pos3相连的关键帧，X2是Pos3看到的三维点，X1是Pos2看到的三维点，这些都属于局部信息，共同参与Bundle  Adjustment。同时，Pos1也可以看到X1，但它和Pos3没有直接的联系，属于Pos3关联的局部信息，参与Bundle  Adjustment，但取值保持不变。Pos0和X0不参与Bundle Adjustment。

因此，参与优化的是下图中红色椭圆圈出的部分，其中红色代表取值会被优化，灰色代表取值保持不变。(u,v)是X在Pos下的二维投影点，即X在Pos下的测量（measurement）。优化的目标是最小重投影误差。

{% asset_img 局部优化.png %}

### 全局优化

#### 对应函数

~~~c++
void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
//调用    
void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
~~~

函数中优化求解器初始化过程如下：

~~~c++
g2o::SparseOptimizer optimizer;
// solver for BA/3D SLAM
g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
// 线性方程求解器 使用的求解库为Eigen
linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
// 稀疏矩阵块求解器
g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
// 梯度下降算法
g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
optimizer.setAlgorithm(solver);
~~~

#### 具体实现

用于单目初始化的`CreateInitialMapMonocular`函数以及闭环优化的`RunGlobalBundleAdjustment`函数（在闭环结束前新开一个线程，进行全局优化，在此之前会`OptimizeEssentialGraph`，论文中说其实这里全局优化提升的精度有限，所以其实可以考虑不使用全局优化）。在全局优化中，所有的关键帧（除了第一帧）和三维点都参与优化。

{% asset_img 全局优化.png %}CorrectLoop

### 闭环处的Sim3位姿优化

#### 对应函数

~~~c++
int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
~~~

如果`DetectLoop`检测到回环，则调用`ComputeSim3`计算相似变换矩阵，再进行下一步的闭环调整。计算相似变换矩阵过程中，在用RANSAC求解过Sim3以及通过Sim3匹配更多的地图点后，对当前关键帧、闭环关键帧以及匹配的地图点进行优化，以获得更准确的Sim3位姿。获得当前关键帧相对于闭环关键帧的Sim3，然后传播到相连关键帧，并调整地图点，从而完成闭环调整。函数中优化求解器初始化过程如下：

~~~c++
g2o::SparseOptimizer optimizer;
//variable size solver
g2o::BlockSolverX::LinearSolverType * linearSolver;
// 线性方程求解器 使用的求解库为Dense
linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
// 稀疏矩阵块求解器
g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
// 梯度下降算法
g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
optimizer.setAlgorithm(solver);
~~~

#### 具体实现

当检测到闭环时，闭环连接的两个关键帧的位姿需要通过Sim3优化（以使得其尺度一致）。优化求解两帧之间的相似变换矩阵，使得二维对应点（feature）的投影误差最小。如下图所示，Pos6和Pos2为一个可能的闭环。通过$(u_{4,2},v_{4,2})$和$(u_{4,6},v_{4,6})$之间的投影误差来优化$S_{6,2}$。

{% asset_img 闭环处优化.png %}

### Sim3上的位姿优化

#### 对应函数

~~~c++
void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
const LoopClosing::KeyFrameAndPose &CorrectedSim3,
const map<KeyFrame *, set<KeyFrame *> > &LoopConnections, const bool &bFixScale)
~~~

EssentialGraph包括所有的关键帧顶点，但是优化边大大减少，包括spanning tree（生成树）和共视权重θ>100的边，以及闭环连接边。用于闭环检测Sim3调整后，闭环调整`CorrectLoop`过程中的优化。函数中优化求解器初始化过程如下：

~~~c++
g2o::SparseOptimizer optimizer;
optimizer.setVerbose(false);
g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

solver->setUserLambdaInit(1e-16);
optimizer.setAlgorithm(solver);
~~~

#### 具体实现

单目SLAM一般都会发生尺度（scale）漂移，因此Sim3上的优化是必要的。相对于SE3，Sim3的自由度要多一个，而且优化的目标是矫正尺度因子，因此优化并没有加入更多的变量（如三维点）。作者在检测到闭环时在Sim3上对所有的位姿进行一次优化。定义Sim3上的残差如下：

$e_{i,j}=log_{Sim3}(S_{ij}S_{jw}S^{−1}_{iw})$

其中$S_{iw}$的初值是尺度为1的Pos i相对于世界坐标系的变换矩阵。$S_{i,j}$为Pos i和Pos j之间的（Sim3优化之前的）相对位姿矩阵，表示$S_{iw}$和$S_{j,w}$之间的测量（measurement）。此处相当于认为局部的相对位姿是准确的，而全局位姿有累计误差，是不准确的。

{% asset_img Sim3上的位姿优化.png %}

### 位姿优化

#### 对应函数

~~~c++
int Optimizer::PoseOptimization(Frame *pFrame)
~~~

#### 具体实现

- 只优化当前帧pose，地图点固定。
- 用于LocalTracking中运动模型跟踪、参考帧跟踪、地图跟踪TrackLocalMap、重定位，每进行过一次PnP投影操作将地图点投影到当前平面上之后，都会进行一次`PoseOptimization`位姿优化，通过BA优化重投影误差。

## ORB-SLAM2中的图

参考：[知乎问题](https://www.zhihu.com/question/42050992)

### Covisibility Graph

共视图，是一个无向有权图（Graph），这个概念最早来自2010的文章[Closing Loops Without Places]。该图中每个顶点就是关键帧，如果两个关键帧有相同的地图点（即它们有共视点），就把这两个顶点连接起来，连接边的权重就是两个关键帧共享的3D点的个数。

### Essential Graph

为了在优化阶段减小计算量，ORB-SLAM2作者提出了**Essential Graph**的概念，主要用它来进行全局优化。它是共视图的子集，即Covisibity Graph的最小生成树（MST）。该图中顶点是关键帧，但只连接某个关键帧和与之拥有最多共享的地图点的关键帧。这样能够连接所有的顶点，但是边会减少很多。

### Spanning Graph

生成树

## 参考资料

1. [ORB-SLAM（五）优化](https://www.cnblogs.com/luyb/p/5447497.html)
2. [ORB-SLAM（十二）优化](https://www.cnblogs.com/shang-slam/p/6483725.html)
3. [ORBSlam2的位姿优化算法](https://blog.csdn.net/chishuideyu/article/details/76013854)