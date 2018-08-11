---
title: ROS学习之actionlib库（３）-使用目标回调方法编写一个简单的行为服务器
date: 2018-03-30 19:06:02
tags:
  - ROS actionlib
categories: ROS
---

-----

这篇文章是有关ROS中actionlib使用的学习内容。

<!--more-->

这一节接着上一节的内容，默认已经创建好程序包（和上一节公用一个程序包，即`actionlib_tutorials`），这里将讲述如何使用目标回调方法(Goal Callback Method)编写一个简单的行为服务器。

# 创建行为消息

在程序包中创建文件`actionlib_tutorials/action/Averaging.action`：

~~~
#goal definition
int32 samples
---
#result definition
float32 mean
float32 std_dev
---
#feedback
int32 sample
float32 data
float32 mean
float32 std_dev
~~~

<!--more--->

# 生成消息文件

添加一些内容到`CMakeList.txt`文件：

~~~cmake
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

add_action_files(
  FILES
  Fibonacci.action
  Averaging.action
  DoDishes.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS actionlib_msgs
)
~~~

运行：

~~~shell
catkin_make #　工作空间下运行
~~~

# 创建行为服务器

创建文件`actionlib_tutorials/src/averaging_server.cpp`：

~~~c++
#include <ros/ros.h>
#include <std_msgs/Float32.h>
// 行为库
#include <actionlib/server/simple_action_server.h>
// 包含从Averaging.action文件中生成的消息
#include <actionlib_tutorials/AveragingAction.h>

class AveragingAction
{
public:
    
  AveragingAction(std::string name) : 
    as_(nh_, name, false),
    action_name_(name)
  {
    //注册目标和反馈回调函数
    as_.registerGoalCallback(boost::bind(&AveragingAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&AveragingAction::preemptCB, this));

    //订阅感兴趣的话题数据 建立一个数据回调，该回调会处理行为
    sub_ = nh_.subscribe("/random_number", 1, &AveragingAction::analysisCB, this);
    as_.start();//行为服务器开启
  }

  ~AveragingAction(void)
  {
  }

  void goalCB()
  {
    // 重置帮助变量
    data_count_ = 0;
    sum_ = 0;
    sum_sq_ = 0;
    // 接收新目标
    goal_ = as_.acceptNewGoal()->samples;
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // 设置行为状态为抢占(preempted)
    as_.setPreempted();
  }

  void analysisCB(const std_msgs::Float32::ConstPtr& msg)
  {
    // 确保行为还没有被取消
    if (!as_.isActive())
      return;
    
    data_count_++;
    feedback_.sample = data_count_;
    feedback_.data = msg->data;
    //处理std_dev和数据含义 
    sum_ += msg->data;
    feedback_.mean = sum_ / data_count_;
    sum_sq_ += pow(msg->data, 2);
    feedback_.std_dev = sqrt(fabs((sum_sq_/data_count_) - pow(feedback_.mean, 2)));
    as_.publishFeedback(feedback_);

    if(data_count_ > goal_) 
    {
      result_.mean = feedback_.mean;
      result_.std_dev = feedback_.std_dev;

      if(result_.mean < 5.0)
      {
        ROS_INFO("%s: Aborted", action_name_.c_str());
        //设置行为状态为崩溃(aborted)
        as_.setAborted(result_);
      }
      else 
      {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // 设置行为状态为成功(succeeded)
        as_.setSucceeded(result_);
      }
    } 
  }

protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tutorials::AveragingAction> as_;
  std::string action_name_;
  int data_count_, goal_;
  float sum_, sum_sq_;
  actionlib_tutorials::AveragingFeedback feedback_;
  actionlib_tutorials::AveragingResult result_;
  ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "averaging");

  AveragingAction averaging(ros::this_node::getName());
  ros::spin();

  return 0;
}
~~~

# 创建行为客户端

创建行为客户端文件`actionlib_tutorials/src/averaging_client.cpp`：

~~~c++
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/AveragingAction.h>
#include <boost/thread.hpp>

void spinThread()
{
  ros::spin();
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_averaging");

  // 创建一个行为客户端
  actionlib::SimpleActionClient<actionlib_tutorials::AveragingAction> ac("averaging");
  boost::thread spin_thread(&spinThread);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  // 发送目标到行为
  actionlib_tutorials::AveragingGoal goal;
  goal.samples = 100;
  ac.sendGoal(goal);

  //等待行为返回
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  // 关闭节点，在退出前加入线程
  ros::shutdown();
  spin_thread.join();

  //exit
  return 0;
}
~~~

# 编译行为

在`CMakeLists.txt`文件末尾添加以下几行： 

~~~cmake
add_executable(averaging_server src/averaging_server.cpp)
target_link_libraries(averaging_server ${catkin_LIBRARIES})

add_executable(averaging_client src/averaging_client.cpp)
target_link_libraries(averaging_client ${catkin_LIBRARIES})
~~~

完整的`CMakeList.txt`文件如下：

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(actionlib_tutorials)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

add_action_files(
  FILES
  Fibonacci.action
  Averaging.action
  DoDishes.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(fibonacci_server src/fibonacci_server.cpp)
target_link_libraries(fibonacci_server  ${catkin_LIBRARIES})

add_executable(fibonacci_client src/fibonacci_client.cpp)
target_link_libraries( fibonacci_client  ${catkin_LIBRARIES})

add_executable(averaging_server src/averaging_server.cpp)
target_link_libraries(averaging_server ${catkin_LIBRARIES})

add_executable(averaging_client src/averaging_client.cpp)
target_link_libraries(averaging_client ${catkin_LIBRARIES})
```

工作空间下执行`catkin_make`命令。

# 运行行为－使用其他节点连接服务器和客户端

## 编写数据节点

创建文件`actionlib_tutorials/scripts/gen_numbers.py`：

~~~python
#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import random
def gen_number():
    pub = rospy.Publisher('random_number', Float32)
    rospy.init_node('random_number_generator', log_level=rospy.INFO)
    rospy.loginfo("Generating random numbers")

    while not rospy.is_shutdown():
        pub.publish(Float32(random.normalvariate(5, 1)))
        rospy.sleep(0.05)

if __name__ == '__main__':
  try:
    gen_number()
  except Exception, e:
    print "done"
~~~

该文件使用一个正态分布生成随机5个数字，并且标准差为1，然后发布数据到/random_number话题。编译该文件节点可运行：`chmod +x gen_numbers.py`。

## 运行行为

1. 终端启动ROS：

```shell
roscore
```

2. 运行数据节点：

~~~shell
rosrun actionlib_tutorials gen_numbers.py 
~~~

3. 查看行为反馈：

```shell
rostopic echo /averaging/feedback
```

执行过程会有一系列的信息输出，最后一条消息是：

~~~shell
header: 
  seq: 100
  stamp: 
    secs: 1522553011
    nsecs: 185810695
  frame_id: ''
status: 
  goal_id: 
    stamp: 
      secs: 1522553006
      nsecs: 117426116
    id: "/test_averaging-1-1522553006.117426116"
  status: 1
  text: "This goal has been accepted by the simple action server"
feedback: 
  sample: 101
  data: 6.57791948318
  mean: 4.95768070221
  std_dev: 1.06043183804
---
~~~

4. 查看行为结果：

```shell
rostopic echo /averaging/result
```

执行完成输出信息：

~~~shell
eric@eric:~$ rostopic echo /averaging/result
header: 
  seq: 0
  stamp: 
    secs: 1522553011
    nsecs: 186018651
  frame_id: ''
status: 
  goal_id: 
    stamp: 
      secs: 1522553006
      nsecs: 117426116
    id: "/test_averaging-1-1522553006.117426116"
  status: 4
  text: ''
result: 
  mean: 4.95768070221
  std_dev: 1.06043183804
---
~~~

5. 运行行为客户端：

```shell
rosrun actionlib_tutorials averaging_client 
```

执行完输出信息：

~~~shell
eric@eric:~$ rosrun actionlib_tutorials averaging_client 
[ INFO] [1522553004.895895005]: Waiting for action server to start.
[ INFO] [1522553006.117383219]: Action server started, sending goal.
[ INFO] [1522553011.186401349]: Action finished: ABORTED
~~~

6. 运行行为服务器：

```shell
rosrun actionlib_tutorials averaging_server
```

7. 查看发布的话题列表（检查行为运行是否正常）：

```shell
rostopic list -v
```

输出信息：

~~~shell
eric@eric:~$ rostopic list -v

Published topics:
 * /turtle1/color_sensor [turtlesim/Color] 1 publisher
 * /averaging/status [actionlib_msgs/GoalStatusArray] 1 publisher
 * /random_number [std_msgs/Float32] 1 publisher
 * /averaging/result [actionlib_tutorials/AveragingActionResult] 1 publisher
 * /rosout [rosgraph_msgs/Log] 6 publishers
 * /averaging/feedback [actionlib_tutorials/AveragingActionFeedback] 1 publisher
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /averaging/goal [actionlib_tutorials/AveragingActionGoal] 1 publisher
 * /averaging/cancel [actionlib_msgs/GoalID] 1 publisher
 * /turtle1/pose [turtlesim/Pose] 1 publisher

Subscribed topics:
 * /averaging/cancel [actionlib_msgs/GoalID] 1 subscriber
 * /averaging/status [actionlib_msgs/GoalStatusArray] 1 subscriber
 * /random_number [std_msgs/Float32] 1 subscriber
 * /averaging/result [actionlib_tutorials/AveragingActionResult] 2 subscribers
 * /rosout [rosgraph_msgs/Log] 1 subscriber
 * /averaging/goal [actionlib_tutorials/AveragingActionGoal] 1 subscriber
 * /averaging/feedback [actionlib_tutorials/AveragingActionFeedback] 2 subscribers
 * /turtle1/cmd_vel [geometry_msgs/Twist] 1 subscriber
~~~

8. 查看行为节点图：

```shell
rqt_graph
```
显式如下节点图：

{% asset_img 节点图.png  %}