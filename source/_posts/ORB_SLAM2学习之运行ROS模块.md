---
title: ORB_SLAM2学习之运行ROS模块
date: 2018-08-12 10:05:55
tags: 
  - ORB_SLAM2
categories:
  - 机器人 
  - SLAM
  - ORB_SLAM2
---

---

这篇文章是有关运行ORB_SLAM2系统ROS模块，包括单目和双目部分的学习内容。

<!--more-->

ORB_SLAM2运行ROS模块需要从相应的话题接收图像用于SLAM系统，对于我这种不方便使用相机进行实时采集图像的渣渣来说，使用数据集图像是很好的选择。因此需要从本地数据集中获取图像，再利用ROS中的话题进行图像的发布和接收。下面的内容将介绍利用ROS进行简单的图像发布和接收操作，以及ORB_SLAM2系统ROS模块运行起来的整个过程。

> 系统环境
>
> - Ubuntu 16.04
> - ROS kinetic

## 基于ROS话题发布、接收图像

### 创建相关软件包

在`catkin_ws/src`目录下新建软件包并编译：

~~~shell
catkin_create_pkg my_image_transport image_transport cv_bridge
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="my_image_transport"
source devel/setup.bash
~~~

### 创建图像发布者程序

新建`my_image_transport/src/my_publisher.cpp`：

~~~c++
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cv::waitKey(30);//不断刷新图像，频率时间为delay，单位为ms
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
~~~

代码解释：

- line 1-4：`ros.h`头文件是所有的ros节点中必须要包含的，下面三个分别是实现图像的发布和订阅，调用opencv库，完成opencv图像格式转化为ROS图像格式所要用到的头文件；
- line 11：告知结点管理器要在`camera/image`话题发布图像消息，参数1是话题名称，话题2是缓冲区大小（即消息队列的长度，在发布图像消息时消息队列的长度只能是1）；
- line 12：根据运行时给定的参数（图像文件的路径）读取图像；
- line 14：将opencv格式的图像转化为ROS所支持的消息类型，从而发布到相应的话题上；
- line 16-21：发布图片消息，使消息类型匹配的节点订阅该消息。

### 创建图像订阅者程序

新建`my_image_transport/src/my_subscriber.cpp`：

~~~c++
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
~~~

代码解释：

- line 6：回调函数，当有新的图像消息到达`camera/image`时，该函数就会被调用；
- line 10：显示捕捉到的图像，其中`cv_bridge::toCvShare(msg, "bgr8")->image`用于将ROS图像消息转化为Opencv支持的图像格式（采用BGR8编码方式）。这部分用法恰好与上一节中发布者节点中的`CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();` 的作用相反
- line 11：刷新图像的频率，实践过程中发现如果注释这一行图像将无法在窗口的显示

### 相关配置文件

1. `CMakeLists.txt`内容

   ~~~cmake
   cmake_minimum_required(VERSION 2.8.3)
   project(my_image_transport)

   ## Compile as C++11, supported in ROS Kinetic and newer
   add_compile_options(-std=c++11)

   find_package(catkin REQUIRED COMPONENTS
     cv_bridge
     image_transport
   )
   find_package(OpenCV REQUIRED)

   set(LIBS
   	${OpenCV_LIBS} 
   	${catkin_LIBRARIES})
   	
   catkin_package(
   #  INCLUDE_DIRS include
   #  LIBRARIES my_image_transport
   #  CATKIN_DEPENDS cv_bridge image_transport
   #  DEPENDS system_lib
   )

   include_directories(
   # include
     ${catkin_INCLUDE_DIRS}
     ${OpenCV_INCLUDE_DIRS}
   )

   add_executable(my_publisher src/my_publisher.cpp)
   target_link_libraries(my_publisher ${LIBS})

   add_executable(my_subscriber src/my_subscriber.cpp)
   target_link_libraries(my_subscriber ${LIBS})
   ~~~

2. `package.xml`文件中添加

   ~~~xml
     <build_depend>opencv2</build_depend>
     <exec_depend>opencv2</exec_depend>
   ~~~

### 编译软件包

~~~shell
cd ~/catkin_ws
catkin_make -DCATKIN_WHITHELIST_PACKAGES="my_image_transport"
~~~

### 运行节点

单独开启一个终端执行`roscore`，启动ros节点管理器。

开启另一个终端，启动发布者节点：

~~~shell
rosrun my_image_transport my_publisher /home/eric/catkin_ws/src/my_image_transport/000.png
~~~

运行订阅者节点：

~~~shell
rosrun my_image_transport my_subscriber
~~~

运行结果如下所示：

{% asset_img 1.png  %}

### 查看当前活动节点及交互情况

查看当前活动节点：

```shell
rosnode list
```

查看各节点交互情况：

~~~powershell
rosrun rqt_graph rqt_graph
~~~

![transport_graph.png](http://wiki.ros.org/image_transport/Tutorials/ExaminingImagePublisherSubscriber?action=AttachFile&do=get&target=transport_graph.png)

可以执行`rosnode kill`命令关闭相关节点。

## ORB_SLAM2 ROS模块结点的编译

在环境变量`ROS_PACKAGE_PATH`中添加`Examples/ROS/ORB_SLAM2`的路径：

~~~shell
echo "source ~/slam/ORB_SLAM2/Examples/ROS/ORB_SLAM2/build/devel/tup.sh" >> ~/.bashrc
~~~

在`~/slam/ORB_SLAM2`目录下执行：

~~~shell
chmod +x build_ros.sh
./build_ros.sh
~~~

等待编译成功。

## ROS Mono

首先在`my_image_transport`目录下创建图像发布者程序`mono_tum.cpp`：

~~~c++
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iostream>
#include <algorithm>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mono_tum");
  if(argc != 2)
  {
      cerr << endl << "Usage: rosrun my_image_transport mono_tum path_to_sequence" << endl;
      return 1;
  }

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);

  vector<string> vstrImageFilenames;
  vector<double> vTimestamps;
  string strFile = string(argv[1])+"/rgb.txt";
  LoadImages(strFile, vstrImageFilenames, vTimestamps);

  int nImages = vstrImageFilenames.size();

  cv::Mat im;
  // double tframe;
  sensor_msgs::ImagePtr msg;
  std_msgs::Header header;
  ros::Rate loop_rate(5);
  for(int ni = 0; ni < nImages; ni++)
  {
    im = cv::imread(string(argv[1])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
    header.stamp = ros::Time(vTimestamps[ni]);

    if(im.empty())
    {
        cerr << endl << "Failed to load image at: "
             << string(argv[1]) << "/" << vstrImageFilenames[ni] << endl;
        return 1;
    }

    cv::waitKey(30);
    msg = cv_bridge::CvImage(header, "bgr8", im).toImageMsg();

    pub.publish(msg);
    cv::waitKey(1);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
~~~

在`CMakeLists.txt`文件中添加：

~~~cmake
add_executable(mono_tum src/mono_tum.cpp)
target_link_libraries(mono_tum ${LIBS})
~~~

下载TUM-rgbd_dataset_freiburg1_xyz数据集，保存`/media/eric/linux/DATA/TUM/rgbd_dataset_freiburg1_xyz`。

测试系统，启动ORB_SLAM2 Mono：

~~~powershell
rosrun ORB_SLAM2 Mono /home/eric/slam/ORB_SLAM2/Vocabulary/ORBvoc.txt ~/slam/ORB_SLAM2/Examples/Monocular/TUM1.yaml
~~~

启动发布者节点：

~~~powershell
rosrun my_image_transport mono_tum /media/eric/linux/DATA/TUM/rgbd_dataset_freiburg1_xyz
~~~

测试效果：

{% asset_img 2.png  %}

## ROS Stereo

首先在`my_image_transport`目录下创建图像发布者程序左相机节点`stereo_left_kitti.cpp`：

~~~c++
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iostream>
#include <algorithm>

using namespace std;

void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageLeft, vector<double> &vTimestamps);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_left_kitti");

  string packagePath = ros::package::getPath("my_image_transport");
  string configPath = packagePath + "//config//stereo_kitti.yaml";

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/camera/left/image_raw", 1);
  
  vector<string> vstrImageLeft;
  vector<double> vTimestamps;
  cv::FileStorage fsSettings(configPath, cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
      cerr << "ERROR: Wrong path to settings" << endl;
      return -1;
  }

  string bagPath = fsSettings["bagPath"];
  LoadImages(bagPath, vstrImageLeft, vTimestamps);

  int nImages = vstrImageLeft.size();

  cv::Mat imLeft;
  sensor_msgs::ImagePtr msg;
  std_msgs::Header header;
  ros::Rate loop_rate(5);
  for(int ni = 0; ni < nImages; ni++)
  {
    imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
    header.stamp = ros::Time(vTimestamps[ni]);

    if(imLeft.empty())
    {
        cerr << endl << "Failed to load image at: "
             << string(vstrImageLeft[ni]) << endl;
        return 1;
    }

    cv::waitKey(30);
    msg = cv_bridge::CvImage(header, "mono8", imLeft).toImageMsg();

    pub.publish(msg);
    cv::waitKey(1);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageLeft, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
    }
}
~~~



右相机节点`stereo_right_kitti`：

~~~c++
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iostream>
#include <algorithm>

using namespace std;

void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_right_kitti");

  string packagePath = ros::package::getPath("my_image_transport");
  string configPath = packagePath + "//config//stereo_kitti.yaml";

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/camera/right/image_raw", 1);

  vector<string> vstrImageRight;
  vector<double> vTimestamps;
  cv::FileStorage fsSettings(configPath, cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
      cerr << "ERROR: Wrong path to settings" << endl;
      return -1;
  }

  string bagPath = fsSettings["bagPath"];
  
  LoadImages(bagPath, vstrImageRight, vTimestamps);

  int nImages = vstrImageRight.size();

  cv::Mat imRight;
  sensor_msgs::ImagePtr msg;
  std_msgs::Header header;
  ros::Rate loop_rate(5);
  for(int ni = 0; ni < nImages; ni++)
  {
    imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
    header.stamp = ros::Time(vTimestamps[ni]);

    if(imRight.empty())
    {
        cerr << endl << "Failed to load image at: "
             << string(vstrImageRight[ni]) << endl;
        return 1;
    }

    cv::waitKey(30);
    msg = cv_bridge::CvImage(header, "mono8", imRight).toImageMsg();

    pub.publish(msg);
    cv::waitKey(1);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
~~~

在`CMakeLists.txt`文件中添加：

```cmake
add_executable(stereo_left_kitti src/stereo_left_kitti.cpp)
target_link_libraries(stereo_left_kitti ${LIBS})

add_executable(stereo_right_kitti src/stereo_right_kitti.cpp)
target_link_libraries(stereo_right_kitti ${LIBS})
```

在`my_image_transport/config`目录下添加配置文件`stereo_kitti.yaml`，保存数据集路径：

~~~yaml
%YAML:1.0
bagPath: "/media/eric/linux/DATA/KITTI/odometry/data_odometry_gray/sequences/03"
~~~

这里需要启动三个节点，每个单独启动会比较麻烦，所以使用ROS Launch文件，同时启动左、右图像发布节点和ORB_SLAM2 Stereo_eric节点。在`my_image_transport/launch`目录下添加ROS节点启动文件`stereo_image_transport.yaml`：

~~~xml
<launch>
  <node name="stereo_left_kitti" pkg="my_image_transport" type="stereo_left_kitti">
  </node>
  <node name="stereo_right_kitti" pkg="my_image_transport" type="stereo_right_kitti">
  </node>
  <node name="Stereo_eric" pkg="ORB_SLAM2" type="Stereo_eric">
  </node>
</launch>
~~~

修改ORB_SLAM2的`ros_stereo.cc`文件，新建为`ros_stereo_eric.cc`：

~~~c++
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>
#include <ros/package.h>
#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo_eric");
    ros::start();

    string packagePath = ros::package::getPath("ORB_SLAM2");
    string configPath = packagePath + "//config//ros_stereo_eric.yaml";

    cv::FileStorage fsSettings(configPath, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
      cerr << "ERROR: Wrong path to settings" << endl;
      return -1;
    }

    string vocPath = fsSettings["vocPath"];
    string settingPath = fsSettings["settingPath"];
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(vocPath,settingPath,ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    string do_rectify = fsSettings["do_rectify"];
    stringstream ss(do_rectify);
	ss >> boolalpha >> igb.do_rectify;//boolalpha函数把bool值显示为true或false

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(settingPath, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {   //cv_ptrLeft->image  is  cv::Mat
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

}
~~~

`ROS/ORB_SLAM2/config`目录下添加配置文件`ros_stereo_eric.yaml`：

~~~yaml
%YAML:1.0
vocPath: "/home/eric/slam/ORB_SLAM2/Vocabulary/ORBvoc.txt"
settingPath: "/home/eric/slam/ORB_SLAM2/Examples/Stereo/KITTI03.yaml"
do_rectify: "false"
~~~

下载KITTI-odometry/data_odometry_gray数据集，保存`/media/eric/linux/DATA/KITTI/odometry/data_odometry_gray`。

测试系统，启动launch文件，同时启动图像发布者节点和ORB_SLAM2 Stereo_eric节点：

```powershell
roslaunch my_image_transport stereo_image_transport.launch
```

测试效果：

{% asset_img 3.png  %}

## 参考资料

1. http://wiki.ros.org/image_transport/Tutorials/PublishingImages
2. http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
3. https://blog.csdn.net/github_30605157/article/details/50990493