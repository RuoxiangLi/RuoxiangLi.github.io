---
title: ROS学习之nodelet
date: 2018-04-04 10:37:16
tags:
  - ROS nodelet
categories: ROS
---

-----

这篇文章是有关ROS中nodelet使用的学习内容。

<!--more--->

nodelet包提供了一种可在同一进程中运行多个算法，并在算法之间进行零拷贝传输的方法。该包提供了实现nodelet所需的nodelet基类以及用于实例化nodelet的NodeletLoader类。

#　Threading Model

一个nodelet管理器有一个线程池，它在管理器中运行的所有节点上共享。在nodelet中运行的代码中有两种可能的线程API。 默认线程模型对所有回调都有一个线程。 还有一个多线程API。Nodelet将在NodeletManager内运行。 nodelet管理器是一个c ++程序，它被设置为监听ROS服务，并且将成为nodelet动态加载的可执行文件。 在这种情况下，我们将运行独立管理器，但在很多情况下，这些管理器将嵌入正在运行的节点中。