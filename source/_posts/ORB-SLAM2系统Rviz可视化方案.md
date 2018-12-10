---
title: ORB_SLAM2系统Rviz可视化方案
date: 2018-10-14 11:14:57
tags: 
  - ORB_SLAM2
  - Rviz
categories: 
  - 机器人
  - SLAM
  - ORB_SLAM2
copyright: true
---

---

本篇记录ORB_SLAM2系统嵌入Rviz可视化模块的方案实现。

<!--more--->

## 概述

ORB-SLAM的地图和定位可视化是通过Rviz进行展示的，而在ORB-SLAM2中，为了不依赖于ROS，ORB-SLAM2的可视化采用了pangolin库。而我的硕士课题有个需求，就是要使ORB-SLAM2的结果和Cartographer的结果同时显示在同一个可视化工具中，而Cartographer是采用Rviz显示的，还定制了专用的Rviz插件。思考再三，决定为ORB-SLAM2重新添加基于Rviz的可视化模块。

## ORB-SLAM2的Rviz可视化

ORB-SLAM中关于Rviz的可视化：

1. ORB-SLAM的Rviz可视化使用单独的一个类来完成可视化信息的发布：MapPublisher类
2. 所有的可视化信息都是Rviz的Mark类型，根据发布的地图点、关键帧、Covisibility Graph、Spanning Tree和相机轨迹，使用了不同的Mark类型。
3. 所有的可视化信息，包括地图、轨迹等都是从ORB-SLAM中的Map类中获取的。
4. 每次获得一帧图像，进行Track后，利用MapPublisher类发布可视化信息。
5. 在配置相应的Rviz，使其可以接收可视化信息。

明白了这几点之后，在ORB-SLAM2中添加Rviz可视化模块就很简单了，主要对源代码做以下改动：

1. 添加MapPublisher类和配置Rviz，可以直接复用ORB-SLAM中的MapPublisher类和Rviz文件；并在每次Track之后（执行完`mpSLAM->TrackStereo()`）利用MapPublisher类发布可视化信息。
2. 为Map类添加返回相关信息的接口。
3. 特别要注意ORB-SLAM2的坐标系下，z轴是朝前的，而Rviz的坐标系下，z轴是朝上的，因此要做相应的转换。
4. 以上改动可以基于[我的这篇文章](http://ttshun.com/2018/08/12/ORB_SLAM2%E5%AD%A6%E4%B9%A0%E4%B9%8B%E8%BF%90%E8%A1%8CROS%E6%A8%A1%E5%9D%97/)完成。

## ORB_SLAM2中的坐标系

ORB_SLAM2中的世界坐标系z轴是朝前的，Rviz坐标系z轴是朝上的，如下图所示。要把ORB_SLAM2中得到的地图（地图点、轨迹、相机、关键帧等）和相机位姿正确显示在Rviz中，需要将数据进行坐标转换。

{% asset_img ORB世界坐标系.png %}

{% asset_img Rviz世界坐标系.png %}

关键帧或相机的显示是以相机坐标系原点为参照，如下图所示，构造了一个四角锥体，以原点为顶点，底面的四个顶点z值大于0。在获取到位姿，将相机或关键帧转换到ORB_SLAM2世界坐标系统之后，再转换到Rviz世界坐标系。

{% asset_img 相机坐标系.png %}

通过以上几个修改，就能在Rviz中显示ORB-SLAM2的地图构建结果和相机位姿了。

## 运行结果

基于之前的内容，修改启动文件如下：

~~~xml
<launch>
  <node name="stereo_left_kitti" pkg="my_image_transport" type="stereo_left_kitti">
  </node>
  <node name="stereo_right_kitti" pkg="my_image_transport" type="stereo_right_kitti">
  </node>
  <node name="Stereo_eric" pkg="ORB_SLAM2" type="Stereo_eric" output="screen">
  </node>
  <node pkg="rviz" type="rviz" name="rviz" output="log" args="-d $(find ORB_SLAM2)/config/rviz.rviz" >
  </node>
</launch>
~~~

其中`rviz.rviz`文件使用的ORB_SLAM中的，只需要将line 50修改为：

~~~xml
    Fixed Frame: ORB_SLAM/World
~~~

执行命令：`roslaunch my_image_transport stereo_image_transport.launch`

运行结果：

{% asset_img 运行结果.png %}

## 使用PCL库显示地图点

目前我的课题考虑基于ORB_SLAM2生成的点云数据生成二维栅格地图再结合Cartographer的地图，调研后发现可以有两种方案。

- 方案一：使用`pointcloud_to_laserscan`包将点云数据转成模拟激光扫描数据，进而可以作为gmapping的输入数据生成二维栅格地图，然后与Cartographer的地图融合。

- 方案二：使用文章[1]中提出的方法直接将点云数据生成二维栅格地图，融合Cartographer的地图。

### 方案一实施

注意：ORB_SLAM2系统`Map`类中的地图点是世界坐标系中的所有地图点，而不是每个关键

ROS`pointcloud_to_laserscan`包`pointcloud_to_laserscan_node`节点将点云`PointCloud`转成2D激光扫描

- 订阅节点：`cloud_in(sensor_msgs/PointCloud2)`
- 发布节点：`scan(sensor_msg/LaserScan)`

参考[here1](https://www.ncnynl.com/archives/201701/1224.html)和[here2](https://www.cnblogs.com/zxouxuewei/p/5307736.html)配置好ROS程序包，在`MapPulisher`类中稍作修改，参照如下内容：

~~~c++
#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>

main (int argc, char **argv)
{
  ros::init (argc, argv, "pcl_create");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;
  // Fill in the cloud data
  cloud.width = 100;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);
  for (size_t i = 0; i < cloud.points.size(); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  //Convert the cloud to ROS message
  pcl::toROSMsg(cloud, output);
  output.header.frame_id = "odom";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
~~~

展示效果：

{% asset_img pcl点云显示.png %}

## 参考资料

[1] 2D Grid Mapping and Navigation with ORB SLAM. Abhineet Kumar Singh, Ali Jahani Amiri.

