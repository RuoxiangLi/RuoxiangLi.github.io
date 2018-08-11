---
title: ROS学习之OpenCV图像、ROS Image转换接口cv_bridge
date: 2018-08-11 21:32:14
tags:
  - ROS cvbridge
categories: ROS
---

这篇文章是有关OpenCV图像与ROS Image转换接口ROS cv_bridge的学习内容。

<!--more--->

由于项目中需要使用ROS消息发布器、接收器分别发布和接收图像消息，一般情况下发布消息之前需要将`cv::Mat`格式的图像转化为ROS Image message，接收到消息后也需要再转化到`cv::Mat`格式。这个过程就需要使用ROS cv_bridge，即是一个ROS和OpenCV库之间提供接口的开发包。

![img](https://images2015.cnblogs.com/blog/976394/201703/976394-20170329110339701-1788396407.png)

## OpenCV图像转ROS Image message

实例代码：

~~~c++
#include <ros/cv_bridge>
//cv::Mat转ROS Image message

~~~



## ROS Image message转OpenCV图像

实例代码：

~~~c++
#include <ros/cv_bridge>
cv_bridge:toCvShare()//ROS Image message转cv::Mat
~~~



toCvShare()函数原型：

~~~c++
CvImageConstPtr cv_bridge::toCvShare(const sensor_msgs::ImageConstPtr & source,
									 const std::string & encoding = std::string()) 	
~~~

函数功能：将`sensor_msgs::Image`类型的message转化为与OpenCV兼容的`cv_bridge::CvImage`类型，



`cv_bridge::CvImage`类（`#include <cv_bridge.h>`）定义：

~~~c++
namespace cv_bridge {
class CvImage
{
//Public Attributes
public:
  std_msgs::Header header;	// 	ROS header.
  std::string encoding;		//	Image encoding ("mono8", "bgr8", etc.)
  cv::Mat image;			// 	Image data for use with OpenCV. 
};
typedef boost::shared_ptr<CvImage> CvImagePtr;
typedef boost::shared_ptr<CvImage const> CvImageConstPtr;
}
~~~



cv_bridge:toCvShare()
http://docs.ros.org/hydro/api/cv_bridge/html/c++/namespacecv__bridge.html#aafa38a1d9be98d9efaefe45fd873133c

https://www.cnblogs.com/li-yao7758258/p/6637079.html

图像编码可以是以下任何一个opencv图像编码：

    8UC[1-4]
    8SC[1-4]
    16UC[1-4]
    16SC[1-4]
    32SC[1-4]
    32FC[1-4]
    64FC[1-4]
介绍集中cvbridge 中常见的数据编码的形式，cv_bridge可以有选择的对颜色和深度信息进行转化。为了使用指定的特征编码，就有下面集中的编码形式：

mono8:  CV_8UC1， 灰度图像

mono16: CV_16UC1,16位灰度图像

bgr8: CV_8UC3,带有颜色信息并且颜色的顺序是BGR顺序

rgb8: CV_8UC3,带有颜色信息并且颜色的顺序是RGB顺序

bgra8: CV_8UC4, BGR的彩色图像，并且带alpha通道

rgba8: CV_8UC4,CV，RGB彩色图像，并且带alpha通道

注：这其中mono8和bgr8两种图像编码格式是大多数OpenCV的编码格式