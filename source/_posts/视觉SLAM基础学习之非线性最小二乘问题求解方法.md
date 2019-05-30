---
title: 视觉SLAM基础学习之非线性最小二乘问题求解方法
date: 2019-05-27 16:57:57
tags: 
  - SLAM
  - 最小二乘
categories: 
  - 机器人
  - SLAM
  - 基础学习
mathjax: true
copyright: true
---
---

《Methods for Non-linear Least Squares Problems》学习记录。
<!--more--->

## Introduction and Definitions

最小二乘问题的目标是找到局部极小解，是全局最优化问题（求目标函数/代价函数的全局最小值）的一个特例。由于全局最优化问题很难求解，习惯上将其简化为查找一个局部的最小解，即找到一个自变量向量，使得目标函数在确定区域内具有极小值。

#### 定义 1.1. 最小二成问题

$f(\mathbf{x})=(f_1(\mathbf{x}),f_2(\mathbf{x}),...,f_m(\mathbf{x}))$

$f_i(\mathbf{x})=f_i(x_1,x_2,...,x_n)=y_i,\quad\mathcal{R}^n\mapsto\mathcal{R},\mathbf{x}\in\mathcal{R}^n,y_i\in\mathcal{R}$

$f(\mathbf{x})$是向量值函数，其自变量处于$n$维空间，值域属于$m$维向量空间，即$f(\mathbf{x})$的值是向量。

$f_i(\mathbf{x})$是分量函数，其自变量是$n$维向量，值域属于实数集。

## 2.1 梯度下降法

梯度下降法是通过梯度方向和步长，直接求解目标函数的最小值时的参数。

越接近最优值时，步长应该不断减小，否则会在最优值附近来回震荡。

## 2.2 牛顿法

牛顿法是求解函数值为0时的自变量取值的方法。

利用牛顿法求解目标函数的最小值其实是转化为求使目标函数的一阶导数为0的参数值。这一转换的理论依据是，函数的极值点处的一阶导数为0。

其迭代过程是在当前位置$x_0​$求该函数的切线，该切线和x轴的角点为$x_1​$，作为新的$x_0​$，重复这个过程，直到角点和函数的零点重合。此时的参数值就是使得目标函数取得极值的参数值。其迭代过程如下图所示。

迭代公式：$x_{k+1}:=x_k+h_n=x_k-H^{-1}\cdot F'(x)=x_k-\frac{F'(x)}{F''(x)}$

$H$为海森矩阵，其实就是目标函数对参数$x​$的二阶导数。

### 总结

牛顿法是通过求解目标函数的一阶导数为0时的参数，进而求出目标函数最小值时的参数。

收敛速度快。

海森矩阵的逆在迭代过程中不断减少，可以起到逐步减少步长的效果。

缺点：海森矩阵的逆计算复杂，代价比较大，因此有了拟牛顿法。

{% asset_img newton.png %}

## 2.3 线性搜索

该方法确定下降搜索的步长（step length）。

文中的图2.1是代价函数随步长变化而变化的情况，横坐标是步长，其值大于0。

## 参考资料

1. 推荐中文解析：https://blog.csdn.net/zhangjunhit/article/details/88883459
2. 梯度下降法和牛顿法比较：https://www.cnblogs.com/lyr2015/p/9010532.html 
3. 文献原文：

http://www2.imm.dtu.dk/pubdb/views/edoc_download.php/3215/pdf/imm3215.pdf