---
title: Cartographer学习一论文阅读
date: 2018-09-12 16:56:16
tags: 
  - Lidar SLAM
  - Cartographer
mathjax: true
categories:
  - 机器人 
  - SLAM
  - Cartographer
---

---

本篇文章记录阅读Google开源Cartographer SLAM系统论文过程中的学习内容。

<!--more-->

## 概况

`Real-time loop closure in 2D LIDAR SLAM`是Google发表在ICRA2016会议上的一篇论文，开源的系统是大名鼎鼎的Cartographer，目前该系统已经有大神改到Cartographer-ROS版本。本文在阅读论文的基础上，参考其他网络博客资料，学习并记录论文的一些要点，通过这个过程希望能够理解论文的核心内容和系统的实现。

### 主要论文

- Real-Time Loop Closure in 2D LIDAR SLAM , ICRA 2016

- Efficient Sparse Pose Adjustment for 2D Mapping  (SPA)

- Real-Time Correlative Scan Matching  (BBS)

### 文章重点

- 第四部分：local 2d slam，主要是将局部地图的scan matching作为一个最小二乘优化问题，由ceres slover解决。       

- 第五部分： closing loop， 采用了 SPA（Sparse Pose Adjustment）进行后端loop  closure。 这个过程中有一个很重要的过程是的scan和 submap的匹配，这里采用了BBS（Branch-and-bound scan  matching）, 它可大幅提高精度和速度。

## 文章贡献

文章的重点不是关注SLAM本身，而是提出了一种基于激光的5cm分辨率实时建图和回环检测方法，减少了计算量，满足实时的大场景地图构建以及大规模的实时优化的性能需求。

为了达到实时闭环检测，文章使用了分支上界法来计算scan-to-submap的匹配作为约束。

## Scan Matching方法介绍

涉及到的相关文献在文后列出，以便以后学习。

1. scan-to-scan matching：基于激光的SLAM中最常用来估计相关位姿的方法。但是非常容易造成累积误差。[1,2,3,4]

2. scan-to-map matching：可以较少累计误差。其中一种方法是使用Gauss-Newton法在线性插值地图上找到局部最优值，前提是获取到了最优的初始位姿估计，这就需要使用有足够高数据获取速率的激光雷达，以保证局部优化的scan-to-map匹配是高效并且鲁棒的。在不稳定的平台上，需要使用惯性测量单元（IMU）将激光扫描投影到水平面上以估计重力方向。[5]

3. pixel-accurate scan matching：可以进一步减少局部误差，但是计算量比较大。这个方法同样可以用来检测回环。[1]

4. 从laser scans中提取特征做匹配，从而减少计算量[4]。

5. histogram-based matching用于回环检测[6]。

6. 用机器学习做laser scan data的特征检测[7]。

## 累积误差处理方式

1. 基于粒子滤波（Particle Filter）的优化。粒子滤波在处理大场景地图时，由于粒子数的极具增长造成资源密集。[8,9,10]

2. 基于位姿图的优化（**Graph-based SLAM**）。与视觉SLAM的位姿图优化大同小异，主要是在观测方程上的区别。[2,11,12,13]

## 系统概述

1. Cartographer是实时的室内建图系统，系统生成5cm分辨率的2D栅格地图地图。

2. laser scans数据被插入到submap中的最佳估计位置，并假定最佳估计位置在短时间内足够准确。

3. scan matching针对最近的submap发生，因此它只取决于最近的scans和全局帧位姿估计中累计的误差。

4. 系统使用pose optimization处理误差累计。

5. submap一旦构建完成，就不会再插入新的scans。submap会用于回环检测过程的scan matching，其实回环检测会将所有已经构建完成的submaps和scans考虑在内。

   如果scan和submap在当前位姿估计下足够接近的话，scan matcher会尝试在submap中寻找回环scan。

   > 当一个新的laser scan加入到地图中时，如果该laser scan的估计位姿与地图中某个submap的某个laser scan的位姿比较接近的话，那么通过某种 scan match策略就会找到该闭环。

   为了减少计算量，Cartographer设计了特殊的策略来找到回环scan。这个策略就是在当前的位姿估计附近设置搜索窗口，在这个搜索窗口内执行branch-and-bound方法来寻找回环scan，如果在搜索窗口中找到了一个足够好的匹配，则会将该匹配作为回环检测约束条件添加到优化问题中。

6. 系统通过使用branch-and-bound方法，并对每个生成的submap预计算出几个栅格地图，保证回环优化的快速完成，快到可以在加入新的scans之前就完成优化，从而保证了一种软实时约束。

7. cartographer的整体架构是典型的 前端建图 （局部地图）+后端优化。 

## 符号说明

- 位姿表示：$\xi=(\xi_x,\xi_y,\xi_{\theta})$

- 扫描scan：$H=\{h_k\}_{k=1,…,K},h_k \in \mathbb{R^2}$

- scan-to-submap变换矩阵：$T_\xi$

- scan-to-submap变换：$T_{\xi }h_k=\underbrace{\left(\begin{matrix}cos\xi_\theta&-sin\xi_\theta\\sin\xi_\theta&cos\xi_\theta\end{matrix} \right)}_{R_\xi}h_k+\underbrace{\left(\begin{matrix}\xi_x\\\xi_y\end{matrix}\right)}_{t_\xi}$

- 概率栅格地图概率值：$M:r\mathbb{Z}\times r\mathbb{Z} \to [p_{min},p_{max}]$

- submap世界坐标系下的位姿（m代表map）：$\Xi^m=\{\xi^m_i\}_{i=1,...,m}$

- scan世界坐标系下的位姿（s代表scan）：$\Xi^s=\{\xi^s_j\}_{j=1,...,n}$

- scan $i$在匹配到的submap $j$坐标系下的位姿：$\xi_{ij}$

- 与scan $i$和submap $j$相对应的协方差矩阵：$\sum_{ij}$

## Local 2D SLAM

系统将局部和全局方法结合到2D SLAM中，两种方法都对LIDAR观测到的位姿进行了优化。这一部分介绍局部地图的scan matching，该问题被构造成最小二乘问题，使用ceres solver解决。

位姿表示为$\xi=(\xi_x,\xi_y,\xi_{\theta})$，这个位姿表示包括$(x,y)$坐标变换（注意这里是二维坐标），以及角度的旋转$\xi_\theta$，对观测位姿的优化实际上就是对scans的优化。平台采用IMU测量重力方向，将水平安装的LIDAR观测到的scans映射到2D平面。 

> #### scan matching
>
> 局部方法中，将每个连续的scan点集和整个地图的一部分进行匹配，就是和submap $M$进行匹配。这个过程中使用了一种非线性的优化方法将submap和scan点集对齐，这也是scan matching的过程。局部方法积累的误差在全局方法消除，即回环检测。

### A. Scans

submap的构建是一个不断将scan和submap坐标系对齐的迭代过程。

一个scan包含一个起点和很多个终点，起点称为scan origin，终点称为scan points，将scan表示为$H=\{h_k\}_{k=1,…,K},h_k \in \mathbb{R^2}$。在scan坐标系下，origin就是坐标原点，scan points就是在scan坐标系下的坐标。

当把一个scans插入到一个submap中时，假设scan坐标系到submap坐标系的坐标转换表示为$T_\xi$（即scan坐标系在submap坐标系下的位姿$\xi$表示为变换矩阵$T_\xi$），即激光传感器在submap坐标系下的位姿。

每个$h_k$在submap坐标系下的坐标为：$T_{\xi }h_k=\underbrace{\left(\begin{matrix}cos\xi_\theta&-sin\xi_\theta\\sin\xi_\theta&cos\xi_\theta\end{matrix} \right)}_{R_\xi}h_k+\underbrace{\left(\begin{matrix}\xi_x\\\xi_y\end{matrix}\right)}_{t_\xi}$

### B. Submaps

一些连续的scans组成submap。采用概率栅格地图的形式表示这些submaps，$M:r\mathbb{Z}\times r\mathbb{Z} \to [p_{min},p_{max}]$，以给定的分辨率（例如5cm）将离散栅格地图点映射到值，这些值可以记作栅格地图点被占用的概率。对于每个栅格地图点，都定义一个相应的pixel，这个piexl是针对于分辨率来说的，对于5cm的分辨率来说，一个pixel相当于一个5*5的方格，那么对应于scan中应该有很多个point，即论文中定义的：pixel包含了所有靠近这个栅格地图点的points。

{% asset_img 栅格地图点和相关像素.png %}

> #### 疑问
>
> 1. 这里栅格地图点和像素的定义不明白？
>
>    答：栅格地图点就是上图中的叉号处，像素定义为叉号周围所有的点的集合，即小方框。
>
> 2. submaps是扫描点的集合？？概率栅格地图表示submaps怎么理解？？
>
>    看源码理解。
>

对于每个要插入submap的scan，都会产生一组称为hits的grid point和一组称为misses的grid point。如下图所示。

{% asset_img submap.png %}

其中阴影带叉的是hit，加入hits集合；阴影不带叉的是miss（scan origin和scan points连线经过的grid points，排除在hits中的），加入misses集合。每个hits中的grid point被赋予初始值$M=p_{hit}$，每个misses中的grid point被赋予初始值$M=p_{miss}$。如果grid point已经有$p$值，则用下述公式更新：

$odds(p)=\frac{p}{1−p}$

$M_{new(x)}=clamp(odds^{−1}(odds(M_{old}(x))⋅odds(p_{hit})))$

> Clamp函数可以将随机变化的数值限制在一个给定的区间[min, max]内，小于min的数值返回min，大于max的数值返回max。

miss集合的更新也是类似的。

### C. Ceres scan matching

将scan插入submap之前，需要通过scan matching对scan的位姿$ \xi $进行优化（优化过程参照当前局部submap），优化过程使用基于Ceres库的scan matcher。scan matcher的任务就是找到一个scan的位姿，能够满足scan points在submap中有最大概率值。这里涉及到的优化问题为非线性最小二乘问题，通过Ceres库进行求解。使得scan在栅格地图中的概率值最大，那么就需要使得（cs）最小，非线性最小二乘问题目标函数构造形式如下：

$\mathop{\arg\min} \limits_{\xi}\sum \limits_{k=1}^K(1−M_{smooth}(T_{\xi}h_k))^2 \qquad\qquad(CS)​$

平滑函数$M_{smooth}$完成了局部submap中概率值从2D到1D的平滑，将值限制在$[0; 1]$范围内，使用双三次插值法（bicubic interpolation）。通常情况下，这种平滑函数的数学优化比栅格地图的分辨率能够提供更好的精度。

对于局部优化问题，一个相对精确的初始估计非常重要。因此如果通过IMU提供角度信息（角速度），来估计scan matching中位姿的旋转分量$\theta$，可以提高优化的准确性。在缺少IMU的情况下，高频率的scan matching或者pixel-accurate scan matching也可以提高准确性，但会增加时间复杂度。

## Closing Loops

> closing loop采用了 SPA（Sparse Pose Adjustment）进行后端loop closure。 这个过程中有一个很重要的过程是的scan和 submap的匹配，采用的算法是BBS（Branch-and-bound scan matching）, 它可大幅提高精度和速度。

### SPA

由于scans只和一个包含最近的几个scans的submap进行匹配，上面所讲的scan matcher会产生比较小的累计误差。

系统通过创建一个个小的submaps实现大的场景地图构建，并使用Sparse Pose Adjustment方法[2]优化所有scans和submaps的位姿，提高精准度。

### BBS

将scans插入处的相关位姿保存在内存中，以便在回环检测优化时使用。此外，所有其他的包含一个scan和一个submap组合，一旦submap不再变化，都会被用于回环检测。一个scan matcher会在后台一直不断的运行，当一个好的scan match被找到，该匹配的约束也会被加入到优化问题（是指回环优化问题）中。

> **上面这一段内容理解的还不是特别清楚？**
>
> 其实是下文要提到的$\xi_{ij}$的求解过程。

### A. 优化问题

回环的优化问题与scan matching的优化问题类似，都是通过构造非线性最小二乘的方式进行的，允许方便地添加残差以考虑附加的数据。每隔几秒，就使用Ceres计算下式的解：

{% raw %}

$\mathop{\arg\min} \limits_{{\Xi^m},{\Xi^s}} \frac{1}{2}\sum \limits_{ij}\rho(E^2(\xi^m_i,\xi^s_j;\sum \limits_{ij},\xi_{ij})) \qquad(SPA)$ [15]

{% endraw %}

$\rho$是一个损失函数，比如Huber loss等。使用损失函数的目的是减少加入到优化问题中的离群点对于系统的影响，这种情况在局部对称的环境，如办公室走廊容易发生，scan matching会将错误的约束加入到优化问题。

$\Xi^m=\{\xi^m_i\}_{i=1,...,m}$是submap的位姿，$\Xi^s=\{\xi^s_j\}_{j=1,...,n}$是scan的位姿，submap位姿和scan位姿都是世界坐标系下的，并且它们之间存在约束条件（这个约束条件是指什么？）用于完成优化。

对于submap $i$和scan $j$，$\xi_{ij}$表示scan在匹配到的submap坐标系下的位姿（$j$在$i$坐标系下的位姿，求解方法在下一节介绍），$\sum_{ij}$是相应的协方差矩阵，这个协方差矩阵可以通过[14]的方式获得，也可以通过(CS)公式获得。残差$E$的计算：

$E^2(\xi^m_i,\xi^s_j;\sum_{ij},\xi_{ij})=e(\xi^m_i,\xi^s_j;\xi_{ij})^T\sum_{ij}^{-1}e(\xi^m_i,\xi^s_j;\xi_{ij})$

$e(\xi^m_i,\xi^s_j;\xi_{ij})=\xi_{ij}-\left(\begin{matrix}R^{-1}_{\xi^m_i}(t_{\xi^m_i}-t_{\xi^s_j}) \\ \xi^m_{i;\theta}-\xi^s_{j;\theta}\end{matrix}\right)$

> ### 协方差
>
> 统计学中，标准差、方差一般是用来描述一维数据的。对于多维数据，一般使用协方差度量两个随机变量关系。
>
> 方差定义：$var(X)=\frac{\sum \limits_{i=1}^{n}(X_i-\overline X)(X_i-\overline X)}{n-1}$
>
> 协方差定义：$cov(X,Y)=\frac{\sum \limits (X_i-\overline X)(Y_i-\overline Y)}{n-1}$
>
> 使用协方差来度量各个维度偏离其均值的程度。如果协方差为正，两个变量是正相关；为负，负相关；为0，无关，即相互独立。
>
> ### 协方差矩阵
>
> 协方差只能处理二维问题，对于多维数据，就需要计算多个协方差。一般使用协方差矩阵组织。
>
> 协方差矩阵定义：$C_{n\times n}=(c_{i,j},c_{i,j}=cov(Dim_i,Dim_j))$
>
> 如三维数据的协方差矩阵为：$C=\left(\begin{matrix}cov(x,x)&cov(x,y)&cov(x,z)\\cov(y,x)&cov(y,y)&cov(y,z)\\cov(z,x)&cov(z,y)&cov(z,z)\end{matrix}\right)$
>
> 协方差矩阵是一个对称的矩阵，而且对角线是各个维度的方差。

### B. Branch-and-bound scan matching

之前提到的回环约束关系$ξij$（scan $j$在submap $i$坐标系中的位姿）就是通过这里的方法得到的，也是整篇论文最核心的地方。首先看一下pixel-accurate match的匹配过程：

$\xi^*=\mathop{\arg\max} \limits_{\xi \in W}\sum\limits_{k=1}^{K}M_{nearest}(T_\xi h_k)\qquad\ \ (BBS)$  [14]

其中$W$是搜索空间（搜索窗口），$M_{nearest}$就是该pixel对应的grid point的M值。之后可以通过(CS)公式进一步提高$\xi$匹配的准确度。

搜索空间和搜索步长的选择是决定pixel-accurate match是否高效的关键。搜索步长的计算方式：

$d_{max}=\mathop{max}\limits_{k=1,...,K} \ \ \| \mathrm{h} _k\|$

$\delta_\theta=arccos(1-\frac{r^2}{2d^2_{max}})$

$w_x=\lceil \frac{W_x}{r} \rceil$,  $w_y=\lceil \frac{W_y}{r} \rceil$,  $w_\theta=\lceil \frac{W_\theta}{\delta_\theta} \rceil$

其中$d_{max}$是所有scan点集中跨度最大的那个值，$Wx=Wy=7m$，$W_θ=30^\circ$，因此搜索空间就可以确定了。此时搜索空间的大小是7m*7m。

$\overline{W}=\{-w_x,...,w_x\}\times\{-w_y,...,w_y\}\times\{-w_\theta,...,w_\theta\}$

$W=\{\xi_0+(rj_x,rj_y,\delta_\theta j_\theta):(j_x,j_y,j_\theta)\in\overline{W}\}​$

有了搜索空间和搜索步长，就可以得到最原始的暴力搜索方式。算法1如下图所示：

{% asset_img 暴力搜索.png %}

为了进一步提高搜索效率，Cartograoher采用了branch and bound approach的方式。branch and bound
approach是一种在问题的解空间树上搜索问题的解的方法，被Google套用在最优位姿的搜索中，从而将无法实时化暴力解优化到可以满足实时化。分支上界法就是每个分支代表一种可能，使用DFS找到最佳位置即可，和算法1解决的是相同的问题。只不过这个节点的score是可能的最大值而已。

分支上界法分为以下几步：**节点选择、分支、计算上限**。对于这三步，论文中有具体讲解。节点选择采用的是DFS，分支算法，采用的是算法2。算法3是将节点选择和分支结合到一起之后的算法。
{% asset_img 算法2.png %}

{% asset_img 算法3.png %}

## 总结

论文阐述了一个2D的SLAM系统，系统采用闭环检测的scan-to-submap matching，同时还有图优化（graph 
optimization）。一个submap的创建使用的是局部的、基于栅格地图的（grid-based）SLAM方法。在后台，所有的点集与相近的submap的匹配使用的是pixel-accurate scan matching的方法，然后建立闭环检测的约束。这个约束图（基于submap和scan pose的）都会周期性的被后台更改。这个操作是采用GPU加速将已完成的submap和当前的submap进行结合。系统图引用的[博客](https://blog.csdn.net/jsgaobiao/article/details/53116042)中的。

{% asset_img 系统图.png %}

Scan是激光扫描的单帧数据，通过累加Scan来构建局部地图（Submap），采用的是grid-based 的SLAM方法。生成约束关系的scan和submap的匹配算法采用的是pixel-accurate scan matching的方法。Cartographer的实现并没有采用滤波方法，而是采用了类似图优化的模型进行Pose estimation，具体的实现是用了Ceres scan matching(Ceres是Google自家的库) 。 这样构造出来的很多很多submap是会产生累计误差的，最后通过Loop closing来消除这些误差，完成闭环。

## 参考资料

1. Real-time loop closure in 2D LIDAR SLAM
2. [google cartographer的论文《real-time loop closure in 2D LIDAR SLAM》翻译](https://blog.csdn.net/lilynothing/article/details/60875825)
3. [cartographer对应论文的琢磨(2)](https://note.youdao.com/share/?id=d8d15963d4577236399aa52c2cd968a7&type=note#/)
4. [Real-Time Loop Closure in 2D LIDAR SLAM 论文笔记](https://zhehangt.github.io/2017/05/01/SLAM/CartographerPaper/)
5. [cartographer算法简析](https://blog.csdn.net/sean_xyz/article/details/68957747)
6. [【SLAM】（一）Google Cartographer的初步尝试](https://blog.csdn.net/jsgaobiao/article/details/53116042)
7. [[转\]浅谈协方差矩阵](https://www.cnblogs.com/chaosimple/p/3182157.html) 

## 相关文献

[1] E. Olson, “M3RSM: Many-to-many multi-resolution scan matching,” in Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), June 2015.
[2] K. Konolige, G. Grisetti, R. Kümmerle, W. Burgard, B. Limketkai, and R. Vincent, “Sparse pose adjustment for 2D mapping,” in IROS, Taipei, Taiwan, 10/2010 2010.
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

[15]Efficient Sparse Pose Adjustment for 2D Mapping

