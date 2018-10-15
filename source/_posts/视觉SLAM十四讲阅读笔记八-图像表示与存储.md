---
title: 视觉SLAM十四讲阅读笔记八-图像表示与存储
date: 2018-10-12 16:48:27
tags: 
  - SLAM基础
  - 读书笔记
mathjax: true
categories: 
  - 机器人
  - SLAM
  - 读书笔记
---

---
这篇文章记录十四讲中代码学习引出的一些内容，主要是计算机中图像的表示和存储相关的。

<!--more-->

## 引出

在阅读十四讲第5讲的源码时，注意到下面这部分代码：

~~~c++
for ( int v=0; v<color.rows; v++ )
    for ( int u=0; u<color.cols; u++ )
    {
        //...
        PointT p ;
        p.x = pointWorld[0];
        p.y = pointWorld[1];
        p.z = pointWorld[2];
        p.b = color.data[ v*color.step+u*color.channels() ];
        p.g = color.data[ v*color.step+u*color.channels()+1 ];
        p.r = color.data[ v*color.step+u*color.channels()+2 ];
        pointCloud->points.push_back( p );
    }
~~~

代码中`color`是彩色图像，9\10\11行从内存中读取像素[u,v]的三个通道值。如下图所示，图像在OpenCV中的表示和存储：

{% asset_img 表示和存储.png %}

其中`step`是行数据长度，即内存中保存图像的一行数据的空间长度。图像矩阵的像素数据按行存储在内存中，通常情况内存足够大的话图像的每一行是连续存放的，也就是在内存上图像的所有数据存放成一行，这种情况在访问时可以提供很大方便，例如上述代码。

## 图像的表示

{% asset_img 图像坐标示意图.png %}

对于一个位于$x,y$处的像素，它在程序中的访问方式是：

~~~c++
unsigned char pixel = image[y][x];
~~~

它对应着灰度值$I(x,y)$的读数。注意$x$和$y$的顺序，如果顺序错误的话，编译器无法提供任何信息，会引起程序运行中的越界错误。

## 参考资料

1. 视觉slam十四讲第5讲
2. [【OpenCV】访问Mat图像中每个像素的值](https://blog.csdn.net/xiaowei_cqu/article/details/7771760)