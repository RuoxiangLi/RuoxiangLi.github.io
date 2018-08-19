---
title: ROS学习之Timer类
date: 2018-03-29 10:46:28
tags:
  - ROS
categories: 
  - 机器人
  - ROS
---

----

这篇文章是有关ROS 定时器类`Timer`类的内容。

<!--more--->

测试程序：

~~~c++
#include "ros/ros.h"
void callback1(const ros::TimerEvent&)
{
  ROS_INFO("Callback 1 triggered");
}

void callback2(const ros::TimerEvent&)
{
  ROS_INFO("Callback 2 triggered");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Timer timer1 = n.createTimer(ros::Duration(0.1), callback1);//每隔0.1秒执行一次回调函数
  ros::Timer timer2 = n.createTimer(ros::Duration(1.0), callback2);//每隔1秒执行一次回调函数
  ros::spin();
  return 0;
}
~~~

