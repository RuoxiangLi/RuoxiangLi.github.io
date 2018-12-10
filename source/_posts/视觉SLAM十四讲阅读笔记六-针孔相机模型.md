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
copyright: true
---

---

这篇文章是视觉SLAM十四讲第5讲阅读过程中总结和记录的学习内容，主要记录针孔相机模型，这是视觉SLAM投影、坐标变换等内容的基础，一定要一次性全搞透。

<!--more--->

## 概述

“机器人如何表示自身位姿”属于SLAM经典模型的运动方程部分。“机器人如何观测外部世界”属于观测方程部分，以相机为主的视觉SLAM中，观测主要是指**相机成像**的过程，这里就涉及到相机的成像原理和成像模型。相机模型中常常涉及到四个坐标系：图像像素坐标系、成像平面坐标系、相机坐标系和世界坐标系。

## 针孔相机模型

{% asset_img 针孔相机模型.png %}

针孔相机模型如上图所示，其中，$O-x-y-x$是相机坐标系，$O$是相机的光心，$f$为相机焦距，$z$轴指向相机的正前方；$O'-x'-y'$是物理成像平面，三维世界中的点$P[X_w,Y_w,Z_w]$，相机坐标系的坐标为$[X_c,Y_c,Z_c]$，经过相机光心投影到物理成像平面上，形成成像点$P'[x,y]$。根据相似三角形原理，有：

$\frac{Z_c}{f}=\frac{X_c}{x} =\frac{Y_c}{y}  \qquad (1)$

整理得：

$x=f\frac{X_c}{Z_c} ,y=f\frac{Y_c}{Z_c} \qquad (2)$

## 像素坐标系

图像像素坐标系通常简称为图像坐标系或者像素坐标系。如下图所示：

{% asset_img 像素坐标系.png %}

像素坐标系的平面为相机的成像平面，原点在图像的左上方，$u$轴向右与$x$轴平行，$v$轴向下与$y$轴平行。像素坐标系的单位是像素(pixel)，也就是我们常说的分辨率。

## 物理成像平面坐标系

物理成像平面在距相机光心一倍焦距的平面上，和像素坐标系处于同一平面，原点是相机光轴与成像平面的交点，即成像平面的中点或者叫principal  point。单位为物理单位，比如毫米。因此成像平面坐标系和像素坐标系只是原点和度量单位不同，两个坐标系之间相差了一个缩放比例和一个原点的平移。

假设某个像素点$P'$坐标为$($u,v$)$，$P'$对应的成像平面坐标为$($x,y$)$，设像素坐标在$u$轴上缩放了$\alpha$倍，在$v$轴上缩放了$\beta$倍（$\alpha,\beta$单位都为像素/米）。成像平面的原点在像素坐标系中的坐标为($c_x$,$c_y$)，则像素坐标系与成像平面坐标系之间有如下转换公式，用到齐次坐标：

$\left\{ \begin{array}{ll} u=\alpha x+c_x\\v=\beta y+c_y \end{array} \right. \Rightarrow \left[\begin{matrix}u\\v\\1\end{matrix}\right]=\left[\begin{matrix}\alpha&0&c_x\\0&\beta&c_y\\0&0&1\end{matrix}\right]\left[\begin{matrix}x\\y\\1\end{matrix}\right] \qquad (3)$

## 相机坐标系

相机坐标系的原点是相机光心，$X_c$和$Y_c$轴与像素坐标系$u$轴和$v$轴平行，$Z_c$轴为相机的光轴。光心到像素平面的距离为焦距$f$。

由图可以看出相机坐标系上的点和成像平面坐标系上的点存在透视投影关系。根据相似三角形关系，成像平面坐标系与相机坐标系之间有如下转换关系：

$\left\{ \begin{array}{ll} x=f\frac{X_c}{Z_c}\\y=f\frac{Y_c}{Z_c} \end{array} \right. \Rightarrow  Z_c\left[\begin{matrix}x\\y\\1\end{matrix}\right]=\left[\begin{matrix}f&0&0&0\\0&f&0&0\\0&0&1&0\end{matrix}\right]\left[\begin{matrix}X_c\\Y_c\\Z_c\\1\end{matrix}\right] \qquad (2)$

根据(2)(3)两式，将$\alpha f$合并成$f_x$，$\beta f$合并成$f_y$，其中$f$称为物理焦距，单位为米，$f_x,f_y$称为像素焦距，单位为像素，得到：

$\left\{ \begin{array}{ll} u=f_x \frac{X_c}{Z_c}+c_x\\v=f_y \frac{Y_c}{Z_c}+c_y \end{array} \right. \qquad (4)$

写成矩阵的形式，用到齐次坐标：

$\left[\begin{matrix}u\\v\\1\end{matrix}\right]=\frac1{Z_c}\left[\begin{matrix}\alpha&0&c_x\\0&\beta&c_y\\0&0&1\end{matrix}\right] \left[\begin{matrix}f&0&0&0\\0&f&0&0\\0&0&1&0\end{matrix}\right]\left[\begin{matrix}X_c\\Y_c\\Z_c\\1\end{matrix}\right]=\frac1{Z_c}\left[\begin{matrix}f_x&0&c_x&0\\0&f_y&c_y&0\\0&0&1&0\end{matrix}\right]\left[\begin{matrix}X_c\\Y_c\\Z_c\\1\end{matrix}\right]\triangleq\frac1{Z_c} KP_c \qquad (5)$

传统习惯上把$Z_c$挪到左侧：

$Z_c\left[\begin{matrix}u\\v\\1\end{matrix}\right]=\left[\begin{matrix}f_x&0&c_x&0\\0&f_y&c_y&0\\0&0&1&0\end{matrix}\right]\left[\begin{matrix}X_c\\Y_c\\Z_c\\1\end{matrix}\right]\triangleq KP_c \qquad(6)$

## 世界坐标系

在环境中选择一个参考坐标系来描述相机和物体的位置，该坐标系称为世界坐标系，单位为m。相机坐标系和世界坐标系之间的关系可以用旋转矩阵$R$和平移向量$t$来描述。假设$P$在世界坐标系下的坐标为$(X_w,Y_w,Z_w)$，则相机坐标系与世界坐标系之间有如下转换关系：

$\left[\begin{matrix}X_c\\Y_c\\Z_c\\1\end{matrix}\right]=\left[\begin{matrix}R&t\\0&1\end{matrix}\right]\left[\begin{matrix}X_w\\Y_w\\Z_w\\1\end{matrix}\right] \qquad (7)$

## 坐标系转换

|               坐标系               | 单位  |       备注       |
| :--------------------------------: | :---: | :--------------: |
|             世界坐标系             |   m   |   描述相机位置   |
| 相机坐标系（可化为归一化相机坐标） |   m   |  原点为相机光心  |
|           成像平面坐标系           |  mm   |  原点为图像中点  |
|             像素坐标系             | pixel | 原点为图像左上角 |

通过上述的四个坐标系可以实现从世界坐标系与像素坐标系之间的转换，如下所示：

$Z_c\left[\begin{matrix}u\\v\\1\end{matrix}\right]=\left[\begin{matrix}\alpha&0&c_x\\0&\beta&c_y\\0&0&1\end{matrix}\right]\left[\begin{matrix}f&0&0&0\\0&f&0&0\\0&0&1&0\end{matrix}\right]\left[\begin{matrix}R&t\\0&1\end{matrix}\right]\left[\begin{matrix}X_w\\Y_w\\Z_w\\1\end{matrix}\right]=\left[\begin{matrix}f_x&0&c_x&0\\0&f_y&c_y&0\\0&0&1&0\end{matrix}\right]\left[\begin{matrix}R&t\\0&1\end{matrix}\right]\left[\begin{matrix}X_w\\Y_w\\Z_w\\1\end{matrix}\right] \qquad (8)$

其中$\left[\begin{matrix}f_x&0&c_x&0\\0&f_y&c_y&0\\0&0&1&0\end{matrix}\right]$称为内参数矩阵$K$，$\left[\begin{matrix}R&t\\0&1\end{matrix}\right]$称为外参数矩阵$T$。

上式简写成：

$Z_c\left[\begin{matrix}u\\v\\1\end{matrix}\right]=KT\left[\begin{matrix}X_w\\Y_w\\Z_w\\1\end{matrix}\right] \qquad (9)$

到此，针孔相机成像模型就搞清楚了。

相机的内参数矩阵往往是已知的并且是固定的，而外参数矩阵在SLAM问题中往往是需要求解的，用于相机的位姿定位。
从世界坐标系到像素坐标系之间的转换关系可知，已知世界坐标系下的三维点坐标，只要已知内外参矩阵，就可以求得像素坐标。而如果已知像素坐标，即使已知内外参矩阵，其世界坐标下的三维点也不是唯一确定的，而是空间的一条直线。即单目相机只能测平面信息，而不能获取深度信息。

## 归一化

在坐标变换过程中通常需要进行归一化处理，得到点在相机归一化平面上的投影，归一化平面是假想的。

三维点在相机坐标系下表示形式为：

$P_c=\left[\begin{matrix}x_c\\y_c\\z_c\end{matrix}\right]$

所以有：

$\left[ \begin{matrix}z_cu\\z_cv\\z_c\end{matrix}\right] = K\left[ \begin{matrix}x_c\\y_c\\z_c\end{matrix}\right]=KPc$

由上式可知，三维点直接经过内参得到的坐标相当于$u,v$坐标在各自的方向上都放大了$z_c$倍。归一化平面是指位于相机前方z=1处的平面上，该平面称为归一化平面。归一化坐标就相当于在$z$的方向上当z=1时用一个平面截断，这时光心与3D点的连线在该面上的点即为该三维点的归一化点，记做$P_{c1}$：

$P_{c1}=\left[\begin{matrix}\frac{x_c}{z_c}\\\frac{y_c}{z_c}\\1\end{matrix}\right]$

由归一化坐标经过内参矩阵后就得到了像素坐标：

$\left[\begin{matrix}u\\v\\1\end{matrix}\right]=K\left[\begin{matrix}\frac{x_c}{z_c}\\\frac{y_c}{z_c}\\1\end{matrix}\right]=KP_{c1}$

因此可以把像素坐标看成对归一化平面上的点进行量化测量的结果。

## 畸变

由于相机透镜的使用，会引入径向畸变，主要分为桶型畸变和枕型畸变 ；由于相机透镜组装过程的误差，导致透镜与成像平面不能严格平行，会引入切向畸变。严格意义上，还需要对这些畸变进行畸变矫正。

## 参考资料

1. 视觉SLAM十四讲第5讲
2. [针孔相机模型](http://zhehangt.win/2017/02/16/SLAM/CameraModel/)
3. [计算机视觉：相机成像原理：世界坐标系、相机坐标系、图像坐标系、像素坐标系之间的转换](https://blog.csdn.net/chentravelling/article/details/53558096)
4. [相机成像模型](https://blog.csdn.net/huangjingwei13/article/details/71439293)