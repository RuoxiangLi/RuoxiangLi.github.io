---
title: ORB-SLAM2学习之源码分析五-一个变量引发的探索
date: 2018-08-20 17:12:28
tags: 
  - ORB_SLAM2
mathjax: true
categories: 
  - SLAM
  - ORB_SLAM2
---

-----

这篇文章是有关ORB_SLAM2系统源码分析的内容，主要记录`KeyFrame::mvuRight`有关的内容。

<!--more--->

## 问题的引出


{% fold +点击查看 %}
~~~c++
void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))//count是map映射，使用key查找元素
        return;
    mObservations[pKF]=idx;//idx是当前地图点在该pKF中对应的关键点的编号

    if(pKF->mvuRight[idx]>=0)//mvuRight？？
        nObs+=2;
    else
        nObs++;//观测到该地图点的关键帧数目
}
~~~
{% endfold %}


在阅读`MapPoint::AddObservation`函数时，注意到`mvuRight`这个变量，程序中通过判断其值对`nObs`变量进行相关操作。一直没搞明白它的意思。终于在[知乎这个问题](https://www.zhihu.com/question/280964049/answer/418426500)找到了答案。这里稍微整理一下内容，并记录下自己由此理解的一些东西。

在构造关键帧时，地图点和关键帧之间的观测关系是很重要的一个点，参考关键帧是哪一帧，该地图点被哪些关键帧观测到，对应的哪些（`idx`）特征点，都是通过一下两个成员变量维护：

~~~c++
//观测到该地图点的关键帧集合
std::map<KeyFrame*,size_t> mObservations;
// 观测到该地图点的关键帧数
int nObs; 
~~~

通过`MapPoint::AddObservation()`函数添加地图点观测，即将能够观测到同一个地图点的关键帧（它们之间存在共视关系）加入到该地图点的`mObservations`集合中。函数中对于`mvuRight`的使用其实是RGB-D和双目模式使用到的”双目信息“之一。

## 分析

首先需要知道的是，RGBD虽然具有深度信息，但是深度图和RGB图不是完全匹配，其中还是有无深度值的区域，中间也存在空洞。在ORB_SLAM优化过程中，无深度信息的特征点是一条射线，就视作Mono模式一同处理；有深度信息的特征点，视作与Stereo模式一同处理。

具体说来，为了让RGB-D模式与Mono和Stereo模式保持一致，在创建图像帧时，会调用`Frame::ComputeStereoFromRGBD`函数，如果有深度就设置`mvuRight`和`mvDepth`的对应值，下面一行代码就是设置**虚拟右图像**上的 u 坐标值，即`mvuRight`。

~~~c++
mvuRight[i] = kpU.pt.x-mbf/d;
~~~

对应公式：

$u_R=u_L-\frac{bf}{d}$

其中$f$是$u$方向上的焦距（即$f_x$），$d$是左图像上的深度，$b$是左右图像基线，$\frac{bf}{d}$是视差（disparity）。

生成 Frame 之后就是调用 `Tracking::Track()`函数，通过特征匹配和局部地图追踪进行初步位姿估计。首先，初始化 `Tracking::StereoInitialization()`只会使用那些有深度的 Stereo 点。初始化完成之后，不论是 `Tracking::TrackWithMotionModel()`与上一普通帧匹配，还是 `Tracking::TrackReferenceKeyFrame()`与上一关键帧匹配，或是`Tracking::Relocalization()`与窗口中所有的关键帧匹配，或者`Tracking::TrackLocalMap`局部地图追踪，都会调用函数 `Optimizer::PoseOptimization()`函数，进行优化。此函数中，对于Mono模式会使用`g2o::EdgeSE3ProjectXYZOnlyPose`，对于Stereo模式会使用` g2o::EdgeStereoSE3ProjectXYZOnlyPose`，具体的代码部分如下。

~~~c++
int Optimizer::PoseOptimization(Frame *pFrame)
{
    //...
	for(int i=0; i<N; i++)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            // Monocular observation  单目模式观测到的地图点
            if(pFrame->mvuRight[i]<0)
            {
            	//...
                g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();
                //...
            }
            else  // Stereo observation  双目观测到的地图点
            {
                //...
                g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();
                //...
            }
        }
    }
    //...
}
~~~

上述两个函数中有`computeError()`函数，分别返回`Vector2d`、`Vector3d`，具体代码如下：

~~~c++
Vector2d EdgeSE3ProjectXYZ::cam_project(const Vector3d & trans_xyz) const{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  return res;
}
~~~

~~~c++
Vector3d EdgeStereoSE3ProjectXYZ::cam_project(const Vector3d & trans_xyz, const float &bf) const{
  const float invz = 1.0f/trans_xyz[2];
  Vector3d res;
  res[0] = trans_xyz[0]*invz*fx + cx;
  res[1] = trans_xyz[1]*invz*fy + cy;
  res[2] = res[0] - bf*invz;
  return res;
}
~~~

`res[0]`、` res[1]` 是$u_L$、$v_L$，`res[2]` 是$u_R=u_L-\frac{bf}{d}$。

## 延伸思考

正如在[源码分析二-初始化](http://ttshun.com/2018/08/16/ORB-SLAM2%E5%AD%A6%E4%B9%A0%E4%B9%8B%E6%BA%90%E7%A0%81%E5%88%86%E6%9E%90%E4%BA%8C-%E5%88%9D%E5%A7%8B%E5%8C%96/)中所述，SLAM系统可以接收各种传感器的图像，不管是双目、单目还是RGB-D，都会将原始图像封装成图像帧，为图像帧的各种信息赋值（使用原始图像进行ORB特征提取在这个过程中完成），之后原始图像就被丢弃，之后的处理过程和原始图像没有关系了。创建图像帧之后进行初始化，再之后的处理过程，除了优化过程（`Optimizer::PoseOptimization`）外，不再区分单目、双目模式**（真是这样吗？？？？）**。

## 参考资料

1. [ORB-SLAM六MapPoint与Map](https://www.cnblogs.com/shang-slam/p/6420575.html)
2. [ORB-SLAM的RGBD为什么要分Mono和Stereo?](https://www.zhihu.com/question/280964049/answer/418426500)