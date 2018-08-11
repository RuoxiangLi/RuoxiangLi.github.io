---
title: ROS学习之消息过滤器messsage_filters
date: 2018-08-08 15:41:19
tags:
  - ROS
categories: ROS

---

-----

这篇文章是有关ROS中messsage_filters使用的学习内容。

<!--more-->

这篇文章是有关ROS中roslaunch使用的学习内容。

https://blog.csdn.net/Start_From_Scratch/article/details/52337689?locationNum=10&fps=1

http://wiki.ros.org/message_filters#Subscriber

时间同步器

TimeSynchronizer过滤器通过其标头中包含的时间戳同步传入通道，并以单个回调的形式输出它们，这些回调采用相同数量的通道。 C ++实现可以同步多达9个通道。

消息同步有两种方式，暂且称之为松同步与紧同步，紧同步是精确的同步，松同步是粗略的同步，分别对应`message_filters::sync_policies::ExactTime` 、`message_filters::sync_policies::ApproximateTime` 