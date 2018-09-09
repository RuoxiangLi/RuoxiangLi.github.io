---
title: 跨平台可视化库Pangolin
date: 2018-09-05 19:30:48
tags: 
  - Pangolin
categories: 
  - SLAM
  - 可视化库
---

---

这篇文章记录SLAM中常用的一种跨平台可视化库的简单使用，包括点、线、面的绘制等。

<!--more-->

## 概述

Pangolin是SLAM系统中常用于可视化的跨平台库，它是对OpenGL进行封装的轻量级的OpenGL输入/输出和视频显示的库。可以用于3D视觉和3D导航的视觉图，可以输入各种类型的视频、并且可以保留视频和输入数据用于debug。其他的可视化库还有MRPT、OpenCV等，都是跨平台的库。Pangolin库中的各种数据类型`人如其名`，很容易理解。下面学习一些平时常用的内容，实例程序可以参考参考资料1。

## 点 

在Pangolin中，点是一切的基础。OpenGL提供了一系列函数指定一个点，它们都以glVertex开头，后面跟一个数字和1~2个字母。例如：`glVertex2d`、`glVertex2f`、`glVertex3f`、`glVertex3fv`等等。其中数字表示参数的个数，字母表示参数的类型：

- s表示16位整数（OpenGL中将这个类型定义为GLshort）
- i表示32位整数（OpenGL中将这个类型定义为GLint和GLsizei）
- f表示32位浮点数（OpenGL中将这个类型定义为GLfloat和GLclampf）
- d表示64位浮点数（OpenGL中将这个类型定义为GLdouble和GLclampd）
- v表示传递的几个参数将使用指针的方式

这些函数除了参数的类型和个数不同以外，功能是相同的。OpenGL的很多函数都是采用类似的形式。   

OpenGL中描述一个面（线、点），采用`glBegin`+`glEnd`命令组的形式：  

~~~c++
glBegin(形状);  　　
	glVertex(顶点1);  　　
	glVertex(顶点2);  　　
	//……  
glEnd();   
~~~

形状可以设为：

- `GL_POINTS`：点
- `GL_LINES`：线
- `GL_LINE_STRIP`：折线
- `GL_LINE_LOOP`：封闭折线
- GL_TRIANGLES：三角形
- `GL_POLYGON`：多边形

`void glPointSize(GLfloat size);`，该函数用于设定点的大小，size必须大于0.0f，默认值为1.0f，单位为“像素”。 

**注意**：对于具体的OpenGL实现，点的大小都有个限度的，如果设置的size超过最大值，则设置可能会有问题。 

## 线

`void glLineWidth(GLfloat width); `，该函数用于设定直线的宽度，其用法跟`glPointSize`类似。画线的形式和画点函数十分类似，不同在于`glBegin()`中的符号常量。使用图元常量`GL_LINES`可连接每一对相邻顶点而得到一组直线段。

## 三角形

画三角形以不同顶点的连接有三种方式，但都是**内部填充**的方式 。

- `GL_TRIANGLES`：如同`GL_LINES`一样，第一个三角形的点是V0,V1,V2，第二个则是V3,V4,V5，即是一个3的倍数。不然最后的一个或两个点不显示。 
- `GL_TRIANGLE_STRIP`：填充方式犹如放弃前一个顶点，如第一个三角形V0,V1,V2，第二个则是V1,V2,V3(舍弃V0)。 
- `GL_TRIANGLE_FAN`：填充方式将永远以V0为起始点，如第一个三角形为V0,V1,V2，第二个则是V0,V2,V3 。

{% asset_img 三角形.png %}

## 其他函数

### `glColor3f`

颜色设置函数，有三个float类型的参数，参数值的范围是[0.0, 1.0]。具体的有：

```c++
glColor3f(0.0, 0.0, 0.0);  //--> 黑色  
glColor3f(1.0, 0.0, 0.0);  //--> 红色  
glColor3f(0.0, 1.0, 0.0);  //--> 绿色  
glColor3f(0.0, 0.0, 1.0);  //--> 蓝色  
glColor3f(1.0, 1.0, 0.0);  //--> 黄色  
glColor3f(1.0, 0.0, 1.0);  //--> 品红色  
glColor3f(0.0, 1.0, 1.0);  //--> 青色  
glColor3f(1.0, 1.0, 1.0);  //--> 白色
```

需要注意的是，如果在`glBegin()`与`glEnd()`函数之间多次连续调用颜色函数，那么只会显示出最后一次调用对应的颜色。例如：

```c++
glBegin(GL_POINTS)  
    glColor3f(0.0, 1.0,  0.0);  //绿色  
    glColor3f(1.0, 1.0,  0.0);  //黄色  
    glVertex(0.25, 0.75, 0.0);  
glEnd();
```

画出来的线是黄色的。

### `glBegin()`

`glBegin()`和`glEnd()`之间可调用的函数：

- `glVertex()`设置顶点坐标 
- `glColor()`设置当前颜色 
- `glIndex()`设置当前颜色表 
- `glNormal()`设置法向坐标
- `glEvalCoord()`产生坐标 
- `glCallList()`,`glCallLists()`执行显示列表 
- `glTexCoord()`设置纹理坐标 
- `glEdgeFlag()`控制边界绘制 
- `glMaterial()`设置材质 

## 参考资料

1. [OpenGL（三）之基础绘制篇](http://www.mamicode.com/info-detail-1139707.html)
2. [OpenGL之glColor3f函数](https://www.jianshu.com/p/de161a954130)
3. [Pangolin学习](https://www.cnblogs.com/shhu1993/p/6814714.html)