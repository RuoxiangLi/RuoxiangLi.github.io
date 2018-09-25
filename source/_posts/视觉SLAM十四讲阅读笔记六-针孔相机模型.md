---
title: 视觉SLAM十四讲阅读笔记六-针孔相机模型
date: 2018-09-01 22:26:11
tags: 
  - SLAM基础
  - 读书笔记
  - 单目
mathjax: true
categories: 
  - 机器人
  - SLAM
  - 读书笔记
---

---

这篇文章是视觉SLAM十四讲第5讲阅读过程中总结和记录的学习内容，主要记录针孔相机模型，这是视觉SLAM投影、坐标变换等内容的基础，一定要一次性全搞透。

<!--more--->

## 概述

“机器人如何表示自身位姿”属于SLAM经典模型的运动方程部分。“机器人如何观测外部世界”属于观测方程部分，以相机为主的视觉SLAM中，观测主要是指**相机成像**的过程，这里就涉及到相机的成像原理和成像模型。相机模型中常常涉及到四个坐标系：图像像素坐标系、成像平面坐标系、相机坐标系和世界坐标系。

## 针孔相机模型

{% asset_img 针孔相机模型.png %}

针孔相机模型如上图所示，其中，O-x-y-x是相机坐标系，O是相机的光心，z轴指向相机的正前方；O‘-x'-y'是物理成像平面，三维世界中的点P经过相机光心投影到物理成像平面上，形成成像点P’。

## 像素坐标系

图像像素坐标系通常简称为图像坐标系或者像素坐标系。如下图所示：

{% asset_img 像素坐标系.png %}

像素坐标系的平面为相机的成像平面，原点在图像的左上方，u轴向右与x轴平行，v轴向下与y轴平行。像素坐标系的单位是像素(pixel)，也就是我们常说的分辨率。

## 成像平面坐标系

成像平面坐标系和像素坐标系在同一个平面上，原点是相机光轴与成像平面的交点，通常情况下是成像平面的中点或者叫principal  point。单位为物理单位，比如毫米。因此成像平面坐标系和像素坐标系只是原点和度量单位不同，两个坐标系之间相差了一个缩放比例和一个原点的平移。

假设某个像素点p坐标为(u,v)，p对应的成像平面坐标为(x,y)，dx和dy表示图像中每个像素在成像平面中的物理尺寸，即上面提到的缩放比例。成像平面的原点在像素坐标系中的坐标为(u0,v0)，则像素坐标系与成像平面坐标系之间有如下转换公式：

$\left\{ \begin{array}{ll} u=\frac{x}{dx}+u_0\\v=\frac{y}{dy}+v_0 \end{array} \right. \Rightarrow  \left[\begin{matrix}u\\v\\1\end{matrix}\right]=\left[\begin{matrix}\frac1{dx}&0&u_0\\0&\frac1{dy}&v_0\\0&0&1\end{matrix}\right]\left[\begin{matrix}x\\y\\1\end{matrix}\right]$

## 相机坐标系

相机坐标系如下图所示：

{% asset_img 相机坐标系.png %}

相机坐标系的原点是光心，xc和yc轴与像素坐标系u轴和v轴平行，zc轴为相机的光轴。光心到像素平面的距离为焦距f。

由图可以看出相机坐标系上的点和成像平面坐标系上的点存在透视投影关系。假设p对应的相机坐标系下的点P的坐标为(Xc,Yc,Zc),则成像平面坐标系与相机坐标系之间有如下转换关系：

$\left\{ \begin{array}{ll} x=f\frac{X_c}{Z_c}\\y=f\frac{Y_c}{Z_c} \end{array} \right. \Rightarrow  Z_c\left[\begin{matrix}x\\y\\1\end{matrix}\right]=\left[\begin{matrix}f&0&0&0\\0&f&0&0\\0&0&1&0\end{matrix}\right]\left[\begin{matrix}X_c\\Y_c\\Z_c\\1\end{matrix}\right]$

## 世界坐标系

在环境中选择一个参考坐标系来描述相机和物体的位置，该坐标系称为世界坐标系。相机坐标系和世界坐标系之间的关系可以用旋转矩阵R和平移向量t来描述。假设P在世界坐标系下的坐标为$(X_w,Y_w,Z_w)$，则相机坐标系与世界坐标系之间有如下转换关系：

$\left[\begin{matrix}X_c\\Y_c\\Z_c\\1\end{matrix}\right]=\left[\begin{matrix}R&t\\0&1\end{matrix}\right]\left[\begin{matrix}X_w\\Y_w\\Z_w\\1\end{matrix}\right]$

## 坐标系转换

通过上述的四个坐标系可以实现从世界坐标系与像素坐标系之间的转换，如下所示：

$Z_c\left[\begin{matrix}u\\v\\1\end{matrix}\right]=\left[\begin{matrix}\frac1{dx}&0&u_0\\0&\frac1{dy}&v_0\\0&0&1\end{matrix}\right]\left[\begin{matrix}f&0&0&0\\0&f&0&0\\0&0&1&0\end{matrix}\right]\left[\begin{matrix}R&t\\0&1\end{matrix}\right]\left[\begin{matrix}X_w\\Y_w\\Z_w\\1\end{matrix}\right]=\left[\begin{matrix}f_x&0&u_0&0\\0&f_y&v_0&0\\0&0&1&0\end{matrix}\right]\left[\begin{matrix}R&t\\0&1\end{matrix}\right]\left[\begin{matrix}X_w\\Y_w\\Z_w\\1\end{matrix}\right]$

其中$\left[\begin{matrix}f_x&0&u_0&0\\0&f_y&v_0&0\\0&0&1&0\end{matrix}\right]$称为内参数矩阵$K$，$\left[\begin{matrix}R&t\\0&1\end{matrix}\right]$称为外参数机坐标系

相机坐标系如下图所示：

{% asset_img 相机坐标系.png %}矩阵$T$。

对于相机的内参数矩阵往往是已知的并且是固定的，而外参数矩阵在SLAM问题中往往是需要求解的，用于相机的位姿定位。
从世界坐标系到像素坐标系之间的转换关系可知，已知世界坐标系下的三维点坐标，只要已知内外参矩阵，就可以求得像素坐标。而如果已知像素坐标，即使已知内外参矩阵，其世界坐标下的三维点也不是唯一确定的，而是空间的一条直线。即单目相机只能测平面信息，而不能获取深度信息。

## 参考资料

1. 视觉SLAM十四讲第5讲
2. [针孔相机模型](http://zhehangt.win/2017/02/16/SLAM/CameraModel/)
3. [计算机视觉：相机成像原理：世界坐标系、相机坐标系、图像坐标系、像素坐标系之间的转换](https://blog.csdn.net/chentravelling/article/details/53558096)