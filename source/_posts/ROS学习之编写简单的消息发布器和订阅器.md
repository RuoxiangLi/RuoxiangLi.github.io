---
title: ROS学习之编写简单的消息发布器和订阅器
date: 2018-03-28 20:57:12
tags: 
  - ROS 
  - C++
  - catkin
  - ROS消息发布器
  - ROS消息订阅器
categories: 
  - 机器人
  - ROS
copyright: true
---

-----

这篇文章是有关ROS中消息发布器和订阅器的学习内容。

<!--more--->

# 创建完成该任务的程序包

~~~shell
cd ~/catkin_ws/src #工作空间catkin_ws
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp #创建程序包
mkdir -p beginner_tutorials/src　#放置所有源代码
~~~

<!--more--->

# 创建消息发布器节点talker

消息发布器节点`talker`将不断在ROS网络中广播消息。

在`beginner_tutorials/src`文件夹下创建`talker.cpp`文件：

~~~c++
//ros.h引用了 ROS 系统中大部分常用的头文件
#include "ros/ros.h"
//引用std_msgs/String 消息, 它存放在 std_msgs package 里
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  //初始化ros，定义节点名字"talker"
  ros::init(argc, argv, "talker");
  //为该进程的节点创建句柄，与ROS系统通信的主要接入点。第一个创建的 NodeHandle 会为节点进行初始化，最后一个销毁的 NodeHandle 则会释放该节点所占用的所有资源。 
  ros::NodeHandle n;
  //告知master，在话题"chatter"上发布<std_msgs::String>类型的数据，chatter_pub用于发布消息
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  //指定自循环的频率，设为10hz
  ros::Rate loop_rate(10);
  //发送数据的次数计数
  int count = 0;
  
  //SIGINT 被触发 (Ctrl-C) 时返回false
  while (ros::ok()){
    //消息对象
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    //终端输出消息
    ROS_INFO("%s", msg.data.c_str());
    //发布消息
    chatter_pub.publish(msg);
    //这里不是必须的，当有回调时需要使用该函数，否则回调函数就不会被调用
    ros::spinOnce();
    //调用 ros::Rate 对象来休眠一段时间以使得发布频率为 10Hz
    loop_rate.sleep();
    
    ++count;
  }
  return 0;
}
~~~

总结流程：

- 初始化 ROS 系统 
- 在 ROS 网络内广播我们将要在 chatter 话题上发布`std_msgs/String`类型的消息 
- 以每秒 10 次的频率在 chatter 上发布消息 

# 创建消息订阅器节点

在`beginner_tutorials/src`文件夹下创建`listener.cpp`文件：

~~~c++
#include "ros/ros.h"
#include "std_msgs/String.h"
//回调函数，接收到chatter话题时就会被调用
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  //告知master订阅chatter话题的消息，当有消息发布到这个话题时，ROS 就会调用回调函数
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  //进入循环，等待回调，所有的回调函数都会被调用，摁下Ctrl-C或者被master关闭时退出
  ros::spin();
  return 0;
}
~~~

流程总结：

- 初始化ROS系统 
- 订阅 `chatter` 话题 
- 进入自循环，等待消息的到达 
- 当消息到达，调用 `chatterCallback()` 函数 

# 编译节点

在`CMakeList.txt`文件中添加：

~~~cmake
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_gencpp)　#不加这句也可以

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_gencpp)　#不加这句也可以
~~~



在工作空间下执行：

~~~shell
catkin_make
~~~

执行完编译命令就会生成两个可执行文件, `talker` 和 `listener`, 默认存储到 `devel space` 目录下，具体是在`~/工作空间/devel/lib/<package name>` 中。

# 运行节点

1. 启动ROS

   ~~~shell
   roscore
   ~~~

2. 启动发布器

   ~~~shell
   cd ~/catkin_ws　#catkin_ws为工作空间
   source ./devel/setup.bash
   rosrun beginner_tutorials talker
   ~~~

3. 启动订阅器

   ~~~powershell
   rosrun beginner_tutorials listener
   ~~~

这样在发布器终端和订阅器终端就会看到相应的输出信息。

![](/home/eric/图片/ros消息发布与订阅.png)

## 推荐阅读

https://blog.csdn.net/weixin_28900531/article/details/79431155

https://blog.csdn.net/u013453604/article/details/49102957