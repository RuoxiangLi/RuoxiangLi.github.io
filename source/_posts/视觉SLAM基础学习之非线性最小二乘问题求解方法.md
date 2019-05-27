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

《Methods for Non-linear Least Squares Problems》学习笔记。
<!--more--->

## Introduction and Definitions

最小二乘问题的目标是找到局部极小解，是全局最优化问题（求目标函数/代价函数的全局最小值）的一个特例。

#### 定义 1.1. 最小二成问题

$f(\mathbf{x})=(f_1(\mathbf{x}),f_2(\mathbf{x}),...,f_m(\mathbf{x}))$

$f_i(\mathbf{x})=f_i(x_1,x_2,...,x_n)=y_i,\quad\mathcal{R}^n\mapsto\mathcal{R},\mathbf{x}\in\mathcal{R}^n,y_i\in\mathcal{R}$

$f(\mathbf{x})$是向量值函数，其自变量处于$n$维空间，值域属于$m$维向量空间，即$f(\mathbf{x})$的值是向量。

$f_i(\mathbf{x})$是分量函数，其自变量是$n$维向量，值域属于实数集。



## 参考资料

1. https://blog.csdn.net/zhangjunhit/article/details/88883459
2. 文献原文：

http://www2.imm.dtu.dk/pubdb/views/edoc_download.php/3215/pdf/imm3215.pdf