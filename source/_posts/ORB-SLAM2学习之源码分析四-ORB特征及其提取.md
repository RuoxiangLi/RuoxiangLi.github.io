---
title: ORB_SLAM2学习之源码分析四-ORB特征及其提取
date: 2018-08-19 13:21:54
tags: 
  - ORB_SLAM2
categories: 
  - 机器人
  - SLAM
  - ORB_SLAM2
---

---

这篇文章是有关ORB_SLAM2系统源码分析的内容，主要记录ORB特征的简单原理和提取有关的内容。

<!--more--->

## ORB特征提取（涉及到Fast关键点、rBRIEF描述子、SIFT特征）

一种直观的提取特征的方式就是在不同图像之间辨认角点，确认它们的对应关系，角点就是所谓的特征。但是单纯的角点无法满足需求，为此设计的许多更加稳定的局部图像特征，如著名的SIFT、SURF、ORB，相比于朴素的角点这些人工设计的特征点具备可重复性、可区分性、高效率、本地性等优点。特征点由**关键点（Key-point）**和**描述子（Descriptor）**组成。如当提及SIFT特征时，是指“提取SIFT关键点，计算SIFT描述子”。

ORB特征包括Oriented FAST关键点和rBRIEF描述子，是在FAST关键点和BRIEF描述描述子基础上改进而来。

- 改进FAST关键点（它没有描述子）：FAST关键点不具有尺度不变性，ORB特征提取方法中，通过构建高斯金字塔，然后在每一层金字塔图像上检测角点，nlevels幅不同比例的图像提取特征点总和作为这幅图像的oFAST（FAST Keypoint Orientation，改进后的FAST关键点）关键点，来实现尺度不变性；FAST关键点也不具有旋转不变性，ORB特征提出使用[矩（moment）](https://www.cnblogs.com/ronny/p/3985810.html)法（灰度质心法）来确定FAST关键点的方向，即通过矩来计算特征点（该图像块的几何中心）以r为半径范围内的质心（该图像块的灰度质心），特征点坐标到质心形成一个向量作为该关键点的方向。
- 改进BRIEF描述子：首先进行旋转不变性改进，加入旋转因子得到steered BRIEF；然后改进特征点描述子的相关性（即描述子可区分性，对误匹配率影响较大），得到rBRIEF特征描述子。

## 在SLAM视觉里程计中的应用--特征点法

{% asset_img SLAM视觉里程计.png %}

{% asset_img SLAM视觉里程计特征点法.png %}

## ORB_SLAM中的特征提取

SLAM初始化过程，首先需要创建图像帧，其关键一步就是对图像进行ORB特征提取，调用函数`ExtractORB()`，其内部调用了`ORBextractor`类中的重载操作符`void operator()`，完成特征提取，提取结果被保存在`Frame`类的成员变量`std::vector<cv:KeyPoint> mvKeys`和`cv:Mat mDescriptors`中，即提取出特征的关键点和描述子。操作符重载的函数如下：

~~~c++
void ORBextractor::operator()( InputArray _image, InputArray _mask, vector<KeyPoint>& _keypoints, OutputArray _descriptors)
{
    if(_image.empty())
        return;

    Mat image = _image.getMat();
    assert(image.type() == CV_8UC1 );

    // Pre-compute the scale pyramid
    ComputePyramid(image);//计算图像尺度金字塔

    vector < vector<KeyPoint> > allKeypoints;
    ComputeKeyPointsOctTree(allKeypoints);//提取图像关键点 并保存在八叉树
    //ComputeKeyPointsOld(allKeypoints);

    Mat descriptors;

    int nkeypoints = 0;
    for (int level = 0; level < nlevels; ++level)
        nkeypoints += (int)allKeypoints[level].size();
    if( nkeypoints == 0 )
        _descriptors.release();
    else
    {
        _descriptors.create(nkeypoints, 32, CV_8U);
        descriptors = _descriptors.getMat();
    }

    _keypoints.clear();
    _keypoints.reserve(nkeypoints);
	//计算每个关键点对应的描述子
    int offset = 0;
    for (int level = 0; level < nlevels; ++level)
    {
        vector<KeyPoint>& keypoints = allKeypoints[level];
        int nkeypointsLevel = (int)keypoints.size();

        if(nkeypointsLevel==0)
            continue;

        // preprocess the resized image 进行高斯模糊，用BORDER_REFLECT_101方法处理边缘
        Mat workingMat = mvImagePyramid[level].clone();
        GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);

        // Compute the descriptors 计算描述子
        Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
        computeDescriptors(workingMat, keypoints, desc, pattern);

        offset += nkeypointsLevel;

        // Scale keypoint coordinates
        if (level != 0)
        {
            float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
            for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                 keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
                keypoint->pt *= scale;
        }
        // And add the keypoints to the output
        _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
    }
}
~~~

ORB特征提取主要过程

- 构建图像尺度金字塔（构造过程另作详细记录）
- 提取ORB关键点、生成八叉树并保存关键点
- 计算每个关键点对应的描述子

提取ORB特征时，每一帧图像共提取1000个特征点，分布在金字塔8层中，层间尺度比例1.2，计算下来金字塔0层大约有217个特征点，7层大约有50个特征点。

同时，为了提取出的特征点能够在图像中分布比较均匀（实际情况中，特征点通常分布得比较集中，这样不利于进行匹配，也不利于精确地求解相机间的位姿从而得到精确的VO轨迹），使用了八叉树（其实是平面上的四叉树）的数据结构来存储提取出的特征点。

这部分内容在`ORBextractor.h`和`ORBextractor.cc`中，代码详细理解可以参考[一起学ORBSLAM2ORB特征点提取](https://blog.csdn.net/qq_30356613/article/details/75231440)这篇文章和参考资料11学习，有时间再详细学习。

## 参考资料（有待详细阅读）

1. [ORB特征提取详解](https://blog.csdn.net/zouzoupaopao229/article/details/52625678)
2. [ORB特征点检测](http://www.cnblogs.com/ronny/p/4083537.html )（墙裂推荐，作者博客里有很多图像特征检测相关的介绍，包括斑点、角点检测和SIFT特征、SURF特征、BRIEF特征描述子等）
4. 视觉SLAM十四讲P132-特征点法（推荐，关于特征点的内容比较清晰整洁）
5. [Fast源码分析](https://blog.csdn.net/zhaocj/article/details/40301561)
5. [Fast API](http://opencv.jp/opencv-2.2_org/cpp/features2d_feature_detection_and_description.html?highlight=fast#StarDetector)
6. [ORBextractor特征提取](https://www.cnblogs.com/shang-slam/p/6421940.html)
7. SIFT：*Distinctive image features from scale-invariant keypoints*
8. SURF：*Surf: Speededup robust features*
9. ORB：*Orb: an efficient alternative to sift or surf*
10. Brief：*Brief: Binary robust independent elementary features*

## 尺度金字塔(Scale pyramid)构建

> 层数(ScaleLevels)：金字塔层数，金字塔中包含的不同尺度的图像层数
>
> 尺度因子(ScaleFactor)：金字塔层与层图像之间的尺度参数，缩放比例

`ORBextractor::ComputePyramid`就是根据尺度因子对图像进行缩放处理。

### 供学习参考资料

1. [尺度空间理论](http://www.cnblogs.com/ronny/p/3886013.html)
2. [OpenCV 图像金字塔](http://www.opencv.org.cn/opencvdoc/2.3.2/html/doc/tutorials/imgproc/pyramids/pyramids.html)
3. [数字图像处理9--尺度空间](https://blog.csdn.net/samkieth/article/details/50407655)

### 