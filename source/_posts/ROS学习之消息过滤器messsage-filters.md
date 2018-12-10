---
title: ROS学习之消息过滤器messsage_filters
date: 2018-08-08 15:41:19
tags:
  - ROS
categories: 
  - 机器人
  - ROS
copyright: true
---

-----

这篇文章是有关ROS中messsage_filters使用的学习内容。

<!--more-->

## 消息滤波器概述

一组消息过滤器，它们接收消息并可以在之后根据过滤器需要满足的条件输出这些消息。

`message_filters`是一个用于`roscpp`和`rospy`的实用程序库。 它集合了许多的常用的消息“过滤”算法。 消息过滤器message_filters类似一个消息缓存，当消息到达消息过滤器的时候，可能并不会立即输出，而是在稍后的时间点里满足一定条件下才输出。

举个例子，比如时间同步器，它接收来自多个源的不同类型的消息，并且仅当它们在每个源上接收到具有相同时间戳的消息时才输出它们，也就是起到了一个消息同步输出的效果。

## Subscriber 订阅者

[C++ message_filters::Subscriber API docs](http://www.ros.org/doc/api/message_filters/html/c++/classmessage__filters_1_1Subscriber.html) 

订阅者过滤器是对ROS订阅的封装，为其他过滤器提供源（source）。订阅者过滤器无法将另一个过滤器的输出作为其输入，而是使用ROS话题作为其输入。即通过过订阅ROS话题，从订阅的话题中获取相关信息作为其输入。

### 输入输出形式

**输入** 

​    无输入连接 

**输出** 

​         C++: `void callback(const boost::shared_ptr<M const>&)`

### 例子（C++）

~~~c++
message_filters::Subscriber<std_msgs::UInt32> sub(nh, "my_topic", 1);
sub.registerCallback(myCallback);
~~~

等同于

~~~c++
ros::Subscriber sub = nh.subscribe("my_topic", 1, myCallback);
~~~

## Policy-Based Synchronizer 基于策略的同步器 [ROS 1.1+]

Synchronizer filter同步滤波器通过包含在其`header`中的时间戳来同步输入通道，并以单个回调的形式经过相同数量的通道输出它们。 C ++实现最多可以同步9个通道。

> 关于header，以sensor_msgs/Image消息为例：
>
> ~~~
> [sensor_msgs/Image]:
> std_msgs/Header header
>   uint32 seq
>   time stamp
>   string frame_id
> uint32 height
> uint32 width
> string encoding
> uint8 is_bigendian
> uint32 step
> uint8[] data
> ~~~

Synchronize滤波器在确定如何同步通道的策略上进行模板化。 

有两种策略：ExactTime和ApproximateTime，理解为松同步与紧同步，紧同步是精确的同步，松同步是粗略的同步，分别对应`message_filters::sync_policies::ExactTime` 、`message_filters::sync_policies::ApproximateTime`。

C++ Header: message_filters/synchronizer.h

### 输入输出形式

#### 输入

**C ++：** 最多9个独立的过滤器，每个过滤器的形式为`void callback（const boost::shared_ptr <M const>＆）`。 支持的过滤器数量由类创建的模板参数的数量决定。  

#### 输出

**C ++：** 对于消息类型M0..M8，`void callback（const boost::shared_ptr <M0 const>＆，...，const boost::shared_ptr <M8 const>＆）`。 参数的数量由类创建的模板参数的数量决定。 

### ExactTime策略

`message_filters::sync_policies::ExactTime`策略要求消息具有完全相同的时间戳以便匹配。 只有在具有相同确切时间戳的所有指定通道上收到消息时，才会调用回调。 从所有消息的`header`域读取时间戳（这是该策略所必需的）。 

C++头文件：message_filters/sync_policies/exact_time.h

~~~c++
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info)
{
  // Solve all of perception here...
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "image", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "camera_info", 1);

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
  // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
~~~

### ApproximateTime 策略

`message_filters::sync_policies::ApproximateTime`策略使用自适应算法来匹配基于其时间戳的消息。 

如果不是所有的消息都有一个标题字段，从中可以确定时间戳，请参见下面的解决方法。 

C++头文件：message_filters/sync_policies/approximate_time.h 

例子(C++)：

~~~c++
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& image1, const ImageConstPtr& image2)
{
  // Solve all of perception here...
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::Image> image1_sub(nh, "image1", 1);
  message_filters::Subscriber<sensor_msgs::Image> image2_sub(nh, "image2", 1);

  typedef  message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
   message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
~~~

如果某些消息的类型不包含`header`字段，则ApproximateTimeSynchronizer默认拒绝添加此类消息。 

## 参考资料

1. https://blog.csdn.net/Start_From_Scratch/article/details/52337689?locationNum=10&fps=1
2. [ROS官网](http://wiki.ros.org/message_filters#Subscriber)
3. https://blog.csdn.net/chishuideyu/article/details/77479758