---
title: lightweight_mapping学习之LocalMeshing
date: 2018-09-04 22:59:48
tags: 
  - lightweight_mapping
  - Delaunay三角剖分
categories: 
  - SLAM
  - navigation
---

---

这篇文章是有关lightwight_mapping项目源码分析的内容，主要记录LocalMesshing模块，该模块涉及CGAL库Delauary三角剖分的知识。

<!---more--->

## 概述

LocalMeshing线程主要任务是使用SLAM优化后的稀疏特征（优化的稀疏三维点云）转化为稠密体积表示。具体来说，SLAM模块提供优化后的稀疏三维点云，LocalMeshing线程使用Delaunay三角剖分算法将三维空间进行细分，使用可视化约束刻画空间。LocalMeshing线程的输入是来自LoopClosing线程传送的关键帧（其实这些关键帧都是在Tracking线程创建的），输出是一系列的三维空间中的点、三角形、边结构，这些信息由可视化线程展示在窗口。

## 疑问

1. 关键帧约束和非关键帧约束区别何在？为何要区分？顶点细化的最后为何要添加这两种约束？？
2. 顶点删除过程，会对新产生的空洞重新三角剖分？？

## 关于Delaunay三角剖分

### Triangulation

三角剖分是代数拓扑学最基本的研究方法。以曲面三角剖分为例，三角剖分需要满足一些条件：

（1）每一块碎片都是曲边三角形（曲边三角形就是以等边三角形的三个顶点为圆心，边长为半径画出的图形，曲面的宽度是等长的）

（2）曲面上任何两个这样的曲边三角形，不能同时相交两条或两条以上的边，只能不相交或者是相切于一条公共边

（3）曲面中所有的都是三角面，且所有三角面的合集是所有点集合的凸包

### Delaunay Triangulation

假设边的集合E中的一条边e（两个端点为a，b）,e如果满足下列条件，则称之为Delaunay边：

存在一个圆经过a,b两点，圆内不包含点集中的任何点，这一特性称为空圆特性。

如果一个曲面的点集的一个三角剖分只包含Delaunay边，那么这样的三角剖分就称为Delaunay三角剖分。

关于Delaunay三角剖分更为直观的定义是：三角剖分中的每个三角形的外接圆的内部都不包含点集中的任何点。

Delaunay三角剖分的算法有翻边算法、逐点插入算法、分割合并算法以及Bowyer-Watson算法等。

下图是Delaunay三角剖分的一个直观示意图：

{% asset_img Delanunay三角剖分.png %}

## LocalMeshing.h

新增文件：

```
delaunay文件夹：FreespaceDelaunayAlgorithm相关的文件
dataStructure.h
FColorMap.h/FColorMap.cpp
LocalMeshing.h/LocalMeshing.cpp
```

增添内容文件：

```
MapDrawer.h MapDrawer.cpp
```

### 重要的数据结构

1. Class `Delaunay3CellInfo`：Delaunay三维剖分四面体信息
2. Class `pointInfo`：顶点信息
3. Class `constraintInfo`：约束信息类，成员包括约束类型、关联的Cell集合

### 约束

约束定义为相机中心与地图点（顶点）之间的线段。每个约束有一个ID，并且保存定义该约束的顶点的ID。

三种类型：

- `CON_KF`：关键帧约束
- `CON_NONKF`：非关键帧约束
- `CON_INFINITY`：deleted 约束（无穷约束？？），程序中并没有用到。

关于添加约束的函数：（分别在什么情况下使用？在后面约束插入函数的介绍中说明）

```
addSetOfConstraints( int vertexID, float pose_x, float pose_y, float pose_z, float time )：直接与点关联的约束
addConstraintKF(int vertexID, KeyFrame *kF)：与关键帧关联的约束
addConstraintNonKF()
```

## LocalMeshing::buildMesh2

该函数是LocalMeshing线程实现功能的主要函数，构建三角剖分过程在该函数中完成。函数主要过程如下：

1. 检查关键帧队列是否为空，不为空则弹出队列头的关键帧，并设置当前关键帧禁止被设为`bad`；
2. 检查当前关键帧的数据可用性、是否为`bad`；
3. 获取当前关键帧的位姿的逆，即`Twc`，由此得到旋转矩阵、平移向量；
4. 获取当前关键帧观测到的地图点`vpMapPoints`；
5. 关键帧图像转成BGR彩色图；
6. 删除外点
   - 从`outlierVertexList`集合获取LocalMapping线程截至目前剔除的外点，保存在`currentOutlierVertexList`，并将`outlierVertexList`（它是LocalMapping线程地图点剔除过程保存外点的接口）外点清空，重新收集；
   - 遍历获取到的外点（也没进行什么有实质内容的操作啊，有什么作用？会在全局顶点集合中查找每一个外点）；
   - 调用顶点集合删除算法删除外点，并得到与外点关联的约束集合。
7. 根据4获取到的`vpMapPoints`收集地图点和约束；
8. 更新地图点和约束
   - 调用顶点插入函数插入顶点，并得到与顶点相关联的约束集合；
   - 处理上一步得到的约束集合；
   - 插入关键帧约束。
9. 检查关键帧变化
   - 获取当前关键帧的临近关键帧集合，对于每一个关键帧进行如下操作：
     - 获取当前关键帧匹配到的地图点；
     - 检查地图点，并收集所有地图点到集合；
   - 检查收集到的地图点变化，如果变化大于阈值则需要更新，将地图点放入待更新集合；
   - 对于待更新集合中的地图点，调用顶点细化函数更新地图点；
   - 对于临近关键帧集合中的每一个关键帧进行如下操作：
     - 如果关键帧是`bad`，则将该关键帧关联的所有约束删除，处理下一个关键帧，否则，继续后面的操作；
     - 获取当前关键帧的位姿；
     - 对于当前关键帧关联的所有约束，进行如下操作： 
       - 获取当前约束关联的顶点，得到其坐标信息；
       - 检查位姿和三维点的变化确定是否需要更新约束，如需要更新则删除当前约束，重新添加关键帧约束。
10. 发布网格地图`tetsToTris_naive2`，创建三维空间点、三角形、边集合，主要过程如下：
    - 初始化三维空间点、三角形、边集合为空；
    - 创建顶点句柄列表，关联到边界顶点（边界顶点与无线远处连接）；
    - 填充顶点列表，创建有限的非边界顶点句柄列表，并创建有用的句柄与顶点的关联映射；
    - 遍历有限面集合，并在网格上添加三角形，创建三角形、边。

## 顶点插入算法

该算法输入是一个顶点的信息，输出是由该顶点引起的冲突Cell关联的约束的集合，由于顶点的插入可能会产生新的四面体，所以还需要重新检查该集合中的约束，将约束关联到四面体（快速搜索与新的约束相交的四面体，并标记为空）。

### 对应函数

```c++
addVertex(int vertexID, float x, float y, float z, set<int>& setUnionedConstraints )
```

### 函数参数

- vertexID：顶点ID
- x,y,z：顶点三维坐标
- setUnionedConstraints：顶点关联的约束集合

### 算法过程

1. 使用传入的参数创建待处理的顶点；
2. 定位顶点的位置；
3. 找到由此顶点引起的冲突Cell集合；
4. 对于每一个冲突Cell，将其从关联的约束的关联Cell列表中删除，同时得到所有冲突Cell关联的约束集合（作为算法输出）；
5. 删除冲突的Cell、插入新顶点并重新三角化空洞产生新的Cell；
6. 将顶点保存到全局顶点集合。

## 约束插入算法

插入约束的函数有三种，分别适用于不同的情况。

1. `addConstraintNonKF()`：适用于没有和关键帧关联的顶点，函数直接输入顶点坐标信息；
2. `addConstraintKF()`：适用于和关键帧关联的顶点，通过关键帧的位姿获取顶点的坐标信息，约束的插入过程与第一种类似，只是增加了和关键帧相关的一些操作，例如该约束分类为`CON_KF`、当前关键帧的约束列表会添加该约束；
3. `addSetOfConstraints`：适用于插入已有的约束，函数输入约束集合，使用约束关联的顶点获取顶点坐标信息。

顶点插入的过程大体类似，只介绍第一种插入函数。

### 对应函数

```c++
addConstraintNonKF( int vertexID, float pose_x, float pose_y, float pose_z, float time )
```

### 函数参数

- vertexID：顶点ID
- pose_x,pose_y,pose_z：顶点三维坐标
- time：时间戳

### 算法过程

1. 通过顶点ID获取顶点信息、顶点操作手柄；
2. 使用传入的参数创建待处理的约束，设为`CON_NONKF`类型；
3. 将约束保存到全局约束集合；
4. 将约束关联到当前顶点；
5. 将约束关联到四面体（快速搜索与新的约束相交的四面体，并标记为空）。

## 约束删除算法

### 对应函数

```c++
removeConstraint( int constraintID )
```

### 函数参数

- constraintID：函数输入，要删除的约束ID

### 算法过程

1. 获取与该约束关联的所有Cell，将该约束与所有Cell消除关联；
2. 获取与该约束关联的顶点，将该约束从顶点的约束列表中删除；
3. 获取该约束关联的关键帧（如果是`CON_KF`类型的约束），将该约束从关键帧的约束列表中删除；
4. 从全局约束集合中删除该约束。

## 顶点删除算法

包括单个顶点的删除和顶点集合的删除两个函数。顶点删除的过程大致可以分为四个步骤：

- 删除与该外点关联的约束
- 收集外点附带的四面体关联的约束集合；
- 删除外点及其附带的四面体，并对产生的空洞重新三角剖分；
- 检查约束集合，约束可能会关联到新产生的四面体。

### 删除顶点

#### 对应函数

```c++
removeVertex_origin( int vertexID )
```

#### 函数参数

- vertexID：函数输入，要删除的外点ID

#### 算法过程

1. 获取该点相关的约束集合（约束ID），调用约束删除算法剔除每一个约束；
2. 获取与该外点相关的所有四面体的`Cell_hande`，对于每一个执行如下操作：
   - 获取与该Cell关联的约束集合；
   - 对于每一个约束，将当前Cell从约束的Cell列表中删除，并收集该约束到集合。
3. 剔除该外点（同时会对新产生的空洞重新三角剖分）；
4. 处理前面收集的约束集合，将约束关联到四面体（快速搜索与新的约束相交的四面体，并标记为空）。

### 删除顶点集合

#### 对应函数

```c++
removeVertexSet(list<int>& currentOutlierVertexList, std::set<int>& setUnionedConstraints)
```

#### 函数参数

- currentOutlierVertexList：函数输入，要删除的外点集合
- setUnionedConstraints：函数输出，与删除外点相关联的约束集合，同顶点插入函数一样，也需要检查约束集合。

#### 算法过程

与顶点删除过程类似，只是多了访问待删除外点集合的循环。

## 顶点细化算法

### 对应函数

```c++
moveVertexSet( const list<int>& moveVertexList )
```

### 函数参数

- moveVertexList：函数输入，待细化（更新坐标）的顶点集合

### 算法过程

1. 对于待细化顶点集合中的每一个顶点，进行如下操作：
   - 获取该顶点的ID、坐标信息（新值），构造`tmp`顶点（这些顶点具备足够的信息，在后续过程进行更新）；
   - 获取该顶点的约束集合，对于集合中的每一个约束进行如下操作：
     - 如果该约束类型为`CON_KF`，则在`tmp`顶点中添加该顶点关联的关键帧位姿信息；
     - 如果该约束类新为`CON_NONKF`，则用该顶点的坐标值（旧值）为`tmp`顶点赋值。
   - 将`tmp`顶点加入`moveList`集合；
2. 调用顶点集合删除函数，将待细化顶点集合中的顶点全部删除，并得到与删除顶点关联的约束集合；
3. 调用顶点添加函数，添加`moveList`集合中的顶点（这一步和上一步会产生新的四面体）；
4. 调用`addSetOfConstraints`函数，处理2中得到的约束集合，与新生成的四面体关联；
5. 添加与`moveList`集合中的顶点关联的关键帧约束；
6. 添加与`moveList`集合中的顶点关联的非关键帧约束。

## 参考资料

1. 论文：Building maps for autonomous navigation using sparse visual SLAM features
2. [CGAL手册](https://doc.cgal.org/latest/Triangulation_3/index.html)
3. 书籍：Computational Geometry Algorithms and Applications(third edition)