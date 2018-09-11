---
title: ORB_SLAM2学习之源码分析八-单目初始化再学习
date: 2018-08-29 21:48:50
tags: 
  - ORB_SLAM2
mathjax: true
categories: 
  - 机器人
  - SLAM
  - ORB_SLAM2
---

---

这篇文章是有关ORB_SLAM2系统源码分析的内容，主要记录单目初始化过程中初始化器的工作，该过程通过对极几何方法计算基础矩阵、单应矩阵进而估计相机运动，再利用三角测量计算特征点的空间位置。

<!--more--->

## 概述

单目初始化的内容定义在`Inlitializer.h`头文件中，该文件定义的内容都是用于单目的，因为双目和RGB-D不需要初始化。地图初始化的目的是得到两帧图像的相对运动$R$，$t$，然后根据相机运动和这两帧匹配好的关键点，三角化初始的三维地图。主要分为五个步骤。

## 主要过程

1. **特征提取与匹配。**在当前帧与参考帧之间（金字塔0层上）提取ORB特征，进行匹配，得到像素坐标下的匹配点，如果匹配点对数过少，就重置参考帧，在`Tracking::MonocularInitialization`中；

2. **并行计算H矩阵和F矩阵。**H矩阵用于特征点都在同平面的情况，使用归一化的DLT（直接线性变换）（normalized DLT）方法、四个匹配点进行计算；F矩阵用于特征点不在同一平面的情况，使用归一化8点法。代码中随机的选8个点（当前帧和参考帧的8个匹配好的点对，一共8对），一共挑选200次（迭代次数），选出得分最高的$H$，$F$；得分计算方法如下：

   {% asset_img 得分计算公式.png %}

   具体解释：$M$可取$F$或$H$。我们以$M=F$为例，首先看到的是，$S_F$是一个累加。因为选了8个点，所以累加8次。$ρ_F$的计算已经给出。最重要的是$d^2$的计算， 是指把第一帧里面的每一个选中的点，重投影到第二帧后得到的重投影误差求平方，以及第二帧里面选中的投影到第一帧里面形成的重投影误差求平方。然后$Γ$ 取5.99，计算出得分。

3. **使用打分机制选择模型。**如果场景接近平面，或者视差较小，可以使用单应性矩阵来解释。根据公式得到$R_H$  $RH>0.40$选择$H$，否则选择$F$。选择模型公式如下：

   {% asset_img 模型打分公式.png %}

4. **恢复出运动和地图的三维结构（sfm）。**利用选择的模型恢复运动和地图点，从$H$或$F$中分解出$R$和$T$，然后根据$R$，$T$三角化匹配好的关键点，得到这些关键点的3D坐标。这就是所谓的初始化地图。分解H矩阵可以恢复出8种姿态，SVD分解E矩阵也可以恢复出4种姿态，通过深度值以及场景的先验信息，一般可以得到唯一满足要求的。但是小视角情况下会出现判断错误的情况出现，因此ORB-SLAM中选择使用这些备选姿态直接三角化出地图点，再通过视角，深度以及重投影误差来判定是否有唯一解，若没有，则放弃，重新回到第一步去初始化。

5. 若初始化成功，则进行**GlobalBundleAdjustment**。

## 重要函数

1. 构造函数：给出参考帧和迭代系数。

   ~~~c++
   Initializer(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);
   ~~~

2. 初始化器：单目初始化调用接口。

   ~~~c++
    bool Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12,
     cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated);
   ~~~

   ### 参数

   - CurrentFrame：输入的参考帧                      
   - vMatches12：输入的匹配
   - R21：输出的旋转矩阵
   - t21：输出的平移向量
   - vP3D：输出的三维地图，即初始化的地图
   - vbTriangulated：标记一组特征点能否进行三角化

   ### 具体实现

   首先找到参考帧和当前帧的配对关系。把这个一一对应关系放入**mvMatches12**变量。然后构造一个200行8列的二维数组mvSets。最后开启两个线程计算$H$，$F$。

3. 单应矩阵和基础矩阵获取算法

   ~~~c++
   void FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
   ~~~

   ~~~c++
   void FindFundamental(vector<bool> &vbInliers, float &score, cv::Mat &F21);
   ~~~

   两个函数分别按照设定的规则获取单应矩阵和基础矩阵，它们的三个参数都是输出。

   ### 具体实现

   两个函数实现是类似的。首先将第一帧图像的关键点和第二帧图像的关键点全部normalize。然后，循环200次（默认的迭代次数），每次循环通过`ComputeH21`（`ComputeF21`）计算出单应矩阵（基础矩阵），并通过`CheckHomography`（`CheckFundamental`）计算它们的得分，找出最高的得分，以及最高得分对应的$H$（$F$）。

4. 单应矩阵和基础矩阵计算函数

   ~~~c++
   cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
   ~~~

   ~~~c++
   cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
   ~~~

   两个函数的参数都是输入，函数返回值就是$H$（$F$）矩阵。

   ### 具体实现

   根据8个点对，由对极约束或者是$H$的约束，获得$F$或$H$。具体可参考对极几何相关的内容。

5. 单应矩阵和基础矩阵得分计算

   ~~~c++
   float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);
   ~~~

   ~~~c++
   float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);
   ~~~

   两个函数输入单应矩阵（基础矩阵），输出相应的得分。

   ### 具体实现

   主要过程2内容的实现。

6. 分解单应矩阵和基础矩阵恢复运动

   ~~~c++
   bool ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                         cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);
   ~~~

   ~~~c++
   bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                         cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);
   ~~~

   ### 参数

   - vbMatchesInliers：输入参数                                          
   - F21：输入参数，计算得到的F矩阵
   - K：输入参数，代表相机参数
   - R21：输出的R
   - t21：输出的t                                                                                      
   - vP3D：输出的3D点
   - vbTriangulated：输出，表示点是不是三角化了
   - minParallax：输入，最小的视差
   - minTriangulated：输入，最少需要三角化的点（50）

   ### 具体实现

   因为从$E$，$H$中直接分解出$R$和$T$是不唯一的，会分解出4种（8种）情况，这两个函数就是根据一些手段和先验知识恢复得到正确唯一的$R$和 $t$。其中`ReconstructF`函数首先通过基础矩阵计算得到本质矩阵，再使用`DecomposeE`计算$E$，然后完成验证解。

7. 三角化

   ~~~c++
   void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);
   ~~~

   三角化实现的函数。

   ### 参数

   - kp1：输入的第一帧关键点
   - kp2：输入与第一帧匹配的第二帧关键点 
   - P1 ：第一帧的相机参数KT
   - P2：第二帧的相机参数KT
   - x3D：输出的三角化的点

8. 正则化

   ~~~c++
   void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);
   ~~~

   ### 参数

   - vKeys：输入的关键点
   - vNormalizedPoints：输出的点
   - T：输出的T，在`void FindHomography(vector<bool> &vbMatchesInliers,  float &score, cv::Mat &H21);`和`void  FindFundamental(vector<bool> &vbInliers, float &score,  cv::Mat &F21);`函数里面用到。

   ### 具体实现

   首次算出输入的所有关键点的平均值`meanX`，`meanY`（关键点`x`，`y`的平均值）；

   然后，每个关键点的`x`，`y`与这个平均值`meanX`，`meanY`做差，将所有得到的差值的绝对值加起来求平均值，得到`meanDevX`，`meanDevY`；

   将上面那些差值除以`meanDevXm`，`meanDevY`，就得到输出的`vNormalizedPoints`。

9. 检测视差、深度、重投影误差

   ~~~c++
   int CheckRT(const cv::Mat &R, const cv::Mat &t, 
               const vector<cv::KeyPoint> &vKeys1, 			
               const vector<cv::KeyPoint> &vKeys2,
               const vector<Match> &vMatches12, vector<bool> &vbInliers,
               const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> 				&vbGood, float &parallax);	
   ~~~

   ### 参数

   - R：输入 1->2                                                      
   - t ：输入 1->2
   - vKeys1：输入         
   - vKeys2：输入                            
   - vMatches12：输入          
   - vbInliers：输入      
   - K：相机参数输入                                              
   - vP3D：输出 
   - th2  ：输入（重投影误差的平方误差）  
   - vbGood ：输出                                                     
   - parallax  ：输出视差

   ### 具体实现

   首先将第一个相机放在原点，构造两个3X4的矩阵，该矩阵等于相机参数$K$乘以该帧的$T$，得到第二个相机中心在第一个相机坐标系（可以看成世界坐标系）下的坐标；然后进入一个循环，循环每次都要计算视差；最后检测视差；分别检测两帧下的深度和重投影误差；返回3D点；返回视差。

   > #### 关于视差
   >
   > 首先计算匹配好的点在世界坐标系下（第一个相机坐标系下）的3D坐标，从第一个相机中心出发连接这个3D点得到一条射线，从第二个相机的中心出发连接这个3D点得到一条射线，这两条射线之间的夹角就是视差(parallax)。

10. 本质矩阵分解

    ~~~c++
    void Initializer::DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
    ~~~

    该函数实现本质矩阵$E$的分解。

## 参考资料

1. [ORB-SLAM2详解（三）自动地图初始化](https://blog.csdn.net/u010128736/article/details/53218140)
2. [ORB-SLAM2学习4  initializer.h](https://www.cnblogs.com/panda1/p/7001105.html)
3. [ORB-SLAM （四）Initializer单目初始化](https://www.cnblogs.com/shang-slam/p/6496411.html)