---
title: ROS学习之OpenCV图像、ROS Image转换接口cv_bridge
date: 2018-08-11 21:32:14
tags:
  - ROS
categories: 
  - 机器人
  - ROS
---

这篇文章是有关OpenCV图像与ROS Image转换接口ROS cv_bridge的学习内容。

<!--more--->

由于项目中需要使用ROS消息发布器、接收器分别发布和接收图像消息，一般情况下发布消息之前需要将`cv::Mat`格式的图像转化为ROS Image message，接收到消息后也需要再转化到`cv::Mat`格式。这个过程就需要使用ROS cv_bridge，即是一个ROS和OpenCV库之间提供接口的开发包。

![img](https://images2015.cnblogs.com/blog/976394/201703/976394-20170329110339701-1788396407.png)



## ROS Image message转OpenCV图像

示例代码：

~~~c++
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr_image;
    try
    {
        cv_ptr_image = cv_bridge::toCvShare(msg, "mono8");
        cv::Mat = cv_ptr_image->image;
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
~~~

### CvImage类

cvbridge定义了一个opencv图像CvImage的类型、包含了编码和ROS的信息头。CvImage包含准确的信息sensor_msgs /image，因此我们可以将两种数据类型进行转换。`cv_bridge::CvImage`类（`#include <cv_bridge.h>`）定义：

```c++
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
```

### toCvShare()函数原型

~~~c++
CvImageConstPtr cv_bridge::toCvShare(const sensor_msgs::ImageConstPtr & source,
									 const std::string & encoding = std::string()) 	
~~~

- 函数功能：将`sensor_msgs::Image`类型的message转化为与OpenCV兼容的`cv_bridge::CvImage`类型；
- 输入：图像消息指针、可选的编码参数（编码是指CvImage的类型）；如果没有输入编码（或更确切地说，空字符串），则目标图像编码将与图像消息编码相同（即与消息发布器发布消息时转化图像时的图像编码）；
- 输出：CvImageConstPtr智能指针，指向CvImage类型的数据。

介绍几种cvbridge中常见的数据编码的形式，cv_bridge可以有选择的对颜色和深度信息进行转化。为了使用指定的特征编码，就有下面集中的编码形式：

- mono8:  CV_8UC1， 灰度图像
- mono16: CV_16UC1,16位灰度图像
- bgr8: CV_8UC3,带有颜色信息并且颜色的顺序是BGR顺序
- rgb8: CV_8UC3,带有颜色信息并且颜色的顺序是RGB顺序
- bgra8: CV_8UC4, BGR的彩色图像，并且带alpha通道
- rgba8: CV_8UC4,CV，RGB彩色图像，并且带alpha通道

注：这其中mono8和bgr8两种图像编码格式是大多数OpenCV的编码格式。

OpenCV图像编码格式：

```
8UC[1-4]
8SC[1-4]
16UC[1-4]
16SC[1-4]
32SC[1-4]
32FC[1-4]
64FC[1-4]
```

### 补充

使用`rosmsg show Header`命令查看消息详细信息：

~~~shell
[std_msgs/Header]:
uint32 seq
time stamp  #ros::Time
string frame_id 
~~~

## OpenCV图像转ROS Image message

示例代码：

```c++
#include <cv_bridge/cv_bridge.h>

int main()
{
    cv::Mat imRight;
    sensor_msgs::ImagePtr msg;
  	std_msgs::Header header;
	imRight = cv::imread("image_path",CV_LOAD_IMAGE_UNCHANGED);
    header.stamp = ros::Time("time_stamp");//不需要传时间戳就不用设置
    if(imRight.empty())
    {
        cerr << endl << "Failed to load image at: "
             << string("image_path") << endl;
        return 1;
    }
    cv::waitKey(30);
    msg = cv_bridge::CvImage(header, "mono8", imRight).toImageMsg();//如果不需要传时间戳，第一参数可以为std_msgs::Header()
}
```

实现这个过程，重点是调用了CvImage类的toImageMsg()函数。

~~~c++
class CvImage
{
  sensor_msgs::ImagePtr toImageMsg() const;

  // Overload mainly intended for aggregate messages that contain
  // a sensor_msgs::Image as a member.
  void toImageMsg(sensor_msgs::Image& ros_image) const;
};
~~~



## 参考资料

1. http://docs.ros.org/hydro/api/cv_bridge/html/c++/namespacecv__bridge.html#aafa38a1d9be98d9efaefe45fd873133c
2. https://www.cnblogs.com/li-yao7758258/p/6637079.html
3. http://docs.ros.org/hydro/api/cv_bridge/html/c++/cv__bridge_8cpp.html