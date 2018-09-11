---
title: lightweight_mapping学习之MapDrawer
date: 2018-09-06 18:27:55
tags: 
  - lightweight_mapping
categories: 
  - 机器人
  - SLAM
  - navigation
---

---

这篇文章是有关lightwight_mapping项目源码分析的内容，主要记录MapDrawer模块新增的内容。

<!--more--->

## 概述

ORB_SLAM2可视化的功能采用的Pangolin库，`MapDrawer`类是对地图可视化的实现，在可视化线程中被调用，类中各功能的实现都是基于Pangolin的，彼此之间过程类似，比较简单。lightwight_mapping项目在该类中新添了四个功能函数：

1. `void DrawVertexes(bool flag);`
2. `void DrawTetrahedra(bool flag, double y_base );`
3. `void DrawSpace(bool flag);`
4. `void DrawMesh(bool flag);`

通过这四个功能函数访问LocalMeshing线程生成的三维空间顶点、三角形、边，并对实现这些信息的可视化，对应的文件为`MapDrawer.h`和`MapDrawer.cpp`。

## 函数功能

1. 该函数通过接口获取LocalMeshing线程生成的三维顶点，使用`GL_POINTS`类型，通过`glVertex3f()`函数将顶点可视化。
2. 该函数通过接口获取LocalMeshing线程生成的三维空间中的三角形（其实就是三个三维点组成的结构），使用`GL_TRIANGLES`类型，通过`glVertex3f()`函数将三角形可视化（这里很有意思，和可视化顶点用的是一个函数～），三角形的各顶点使用的不同颜色。
3. 该函数通过接口获取LocalMeshing线程生成的三维空间中的边（其实就是两个三维点组成的结构），使用`GL_LINES`类型，通过`glVertex3f()`函数将三维边可视化，两个顶点使用的相同颜色。
4. 该函数通过接口获取LocalMeshing线程生成的三维空间中的三角形，使用`GL_LINES`类型，通过`glVertex3f()`函数将三角形可视化，这里三角形的三个顶点使用的同一种颜色。

## 参考资料

1. 无