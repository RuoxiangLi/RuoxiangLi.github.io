---
title: ROS学习之actionlib库（２）-使用Execute Callback编写一个简单的行为服务器
date: 2018-03-30 10:42:46
tags:
  - ROS actionlib
categories: 
  - 机器人
  - ROS
---
-----
这篇文章是有关ROS中actionlib使用的学习内容。

<!--more-->

# 创建行为消息

行为消息自动从`.action`文件生成，该文件放置在程序包的`action`目录下，它定义行为消息的目标、结果和行为反馈话题的类型和格式。下面是一个例子。

在程序包中创建文件`actionlib_tutorials/action/Fibonacci.action`：

```
#goal definition
int32 order
---
#result definition
int32[] sequence
---
#feedback
int32[] sequence
```

# 生成消息文件

使用编辑好的`.action`文件生成`.msg`消息文件，有两种方式，笔者认为手动生成方式其实不是必须的，只是提供了一种生成消息文件的方式而已，一般实践过程中只需要在`CMakeList.txt`文件添加必要的信息，自动生成消息文件。

- 通过设置`CMakeList.txt`文件在编译过程中自动生成，生成的`.msg`消息文件会自动放在 `工作空间/devel/share/程序包名/msg/` 路径下；
- 使用`generation.py`脚本手动生成，生成的`.msg`消息文件可以自定义放置的目录，例如可以放在`工作空间/src/程序包名/msg/` 路径下，这时可以在当前程序包下执行命令`rosrun actionlib_msgs genaction.py -o msg/ action/Fibonacci.action`。

## 手动生成

```shell
roscd actionlib_tutorials
rosrun actionlib_msgs genaction.py -o msg/ action/Fibonacci.action
```

`genaction.py`文件位于`/opt/ros/kinetic/lib/actionlib_msgs/`目录下。

会出现如下提示信息：

```shell
Generating for action Fibonacci
```

## 自动生成

在编译过程中自动生成消息需要添加一些内容到`CMakeList.txt`文件。

```cmake
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
```

运行：

```shell
catkin_make #　工作空间下运行
```

使用如下命令就可以看到自动生成的`.msg`和`.h`文件：

```shell
ls devel/share/actionlib_tutorials/msg/
ls devel/include/actionlib_tutorials/
```

# 创建行为服务器

创建文件`actionlib_tutorials/src/fibonacci_server.cpp`：

```c++
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>

class FibonacciAction
{
protected:

  ros::NodeHandle nh_;//　将会传递到行为服务器中
  actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  actionlib_tutorials::FibonacciFeedback feedback_;//　反馈消息
  actionlib_tutorials::FibonacciResult result_;//　结果消息

public:
  FibonacciAction(std::string name) :
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }//　行为构造函数构造行为服务器as_,它会得到一个节点句柄（node handle）、行为名称和选择一个运行回调函数（executeCB）参数。

  ~FibonacciAction(void)
  {
  }
  void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr &goal)//　传递一个指向目标消息的指针，它是一个boost共享指针
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // 开始执行行为服务器
    for(int i=1; i<=goal->order; i++)
    {
      // 检测一个客户端请求是否抢占当前目标。体现行为服务器的一个重要组成部分：允许行为客户端请求取消当前目标执行
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();//发出该行为已经被用户请求抢占信号
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);//设置检查抢占请求服务器的等级到服务器系统
      // 发布反馈：Fibonacci序列赋值给feedback变量，然后通过行为服务器提供的反馈频道发布出去
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);//　一旦行为完成计算Fibonacci序列，通知行为客户端操作设置成功
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");

  FibonacciAction fibonacci("fibonacci");//创建行为
  ros::spin();//spin节点，行为会运行并等待接收目标

  return 0;
}
```

# 创建行为客户端

创建行为客户端文件`actionlib_tutorials/src/fibonacci_client.cpp`：

```c++
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // 创建行为客户端
  // 成功会开启客户端创建自己的线程
  actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);

  ROS_INFO("Waiting for action server to start.");
  //等待行为服务器开启
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // 发送目标到行为服务器
  actionlib_tutorials::FibonacciGoal goal;
  goal.order = 20;
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

  //exit
  return 0;
}
```

# 编译行为

在`CMakeLists.txt`文件末尾添加以下几行： 

```cmake
add_executable(fibonacci_server src/fibonacci_server.cpp)
target_link_libraries(fibonacci_server ${catkin_LIBRARIES})

add_executable(fibonacci_client src/fibonacci_client.cpp)
target_link_libraries(fibonacci_client ${catkin_LIBRARIES})
```

完整的`CMakeList.txt`文件如下：

~~~cmake
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
~~~

工作空间下执行`catkin_make`命令。

其实还需要在`package.xml`中添加如下信息，只不过在最开始创建程序包的时候`catkin_create_pkg`命令添加了对`actionlib`和`actionlib_msgs`的依赖，生成的程序包文件`package.xml`中已经自动添加了这些信息，不需要手动添加了。完美～

~~~xml
  <build_depend>actionlib</build_depend>
  <build_depend>actionlib_msgs</build_depend>
  <build_export_depend>actionlib</build_export_depend>
  <build_export_depend>actionlib_msgs</build_export_depend>
  <exec_depend>actionlib</exec_depend>
  <exec_depend>actionlib_msgs</exec_depend>
~~~

# 运行行为－连接服务器和客户端

1. 终端启动ROS：

```shell
roscore
```

2. 查看行为反馈：

```shell
rostopic echo /fibonacci/feedback
```

会有一系列的输出信息，最后的一条信息为：

~~~shell
---
header: 
  seq: 19
  stamp: 
    secs: 1522552545
    nsecs: 242077849
  frame_id: ''
status: 
  goal_id: 
    stamp: 
      secs: 1522552526
      nsecs: 241284287
    id: "/test_fibonacci-1-1522552526.241284287"
  status: 1
  text: "This goal has been accepted by the simple action server"
feedback: 
  sequence: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 233, 377, 610, 987, 1597, 2584, 4181, 6765, 10946]
~~~

3. 查看行为结果：

```shell
rostopic echo /fibonacci/result
```

执行完成输出信息：

~~~shell
eric@eric:~$ rostopic echo /fibonacci/result
WARNING: topic [/fibonacci/result] does not appear to be published yet
header: 
  seq: 0
  stamp: 
    secs: 1522552546
    nsecs: 242312501
  frame_id: ''
status: 
  goal_id: 
    stamp: 
      secs: 1522552526
      nsecs: 241284287
    id: "/test_fibonacci-1-1522552526.241284287"
  status: 3
  text: ''
result: 
  sequence: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 233, 377, 610, 987, 1597, 2584, 4181, 6765, 10946]
---
~~~

4. 运行行为客户端：

```shell
rosrun actionlib_tutorials fibonacci_client 
```
执行完成的输出信息：

~~~shell
eric@eric:~$ rosrun actionlib_tutorials fibonacci_client
[ INFO] [1522552522.129016184]: Waiting for action server to start.
[ INFO] [1522552526.241189187]: Action server started, sending goal.
[ INFO] [1522552546.242949052]: Action finished: SUCCEEDED
~~~

5. 运行行为服务器：

```shell
rosrun actionlib_tutorials fibonacci_server
```

执行完成的输出信息：

~~~shell
eric@eric:~rosrun actionlib_tutorials fibonacci_server
[ INFO] [1522552526.242112000]: fibonacci: Executing, creating fibonacci sequence of order 20 with seeds 0, 1
[ INFO] [1522552546.242205514]: fibonacci: Succeeded
~~~

6. 查看发布的话题列表（检查行为运行是否正常）：

~~~shell
rostopic list -v
~~~

得到如下信息：

~~~shell
eric@eric:~$ rostopic list -v

Published topics:
 * /turtle1/color_sensor [turtlesim/Color] 1 publisher
 * /fibonacci/feedback [actionlib_tutorials/FibonacciActionFeedback] 1 publisher
 * /fibonacci/cancel [actionlib_msgs/GoalID] 1 publisher
 * /rosout [rosgraph_msgs/Log] 5 publishers
 * /fibonacci/goal [actionlib_tutorials/FibonacciActionGoal] 1 publisher
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /fibonacci/status [actionlib_msgs/GoalStatusArray] 1 publisher
 * /fibonacci/result [actionlib_tutorials/FibonacciActionResult] 1 publisher
 * /turtle1/pose [turtlesim/Pose] 1 publisher

Subscribed topics:
 * /fibonacci/feedback [actionlib_tutorials/FibonacciActionFeedback] 2 subscribers
 * /rosout [rosgraph_msgs/Log] 1 subscriber
 * /fibonacci/cancel [actionlib_msgs/GoalID] 1 subscriber
 * /fibonacci/goal [actionlib_tutorials/FibonacciActionGoal] 1 subscriber
 * /fibonacci/status [actionlib_msgs/GoalStatusArray] 1 subscriber
 * /fibonacci/result [actionlib_tutorials/FibonacciActionResult] 2 subscribers
 * /turtle1/cmd_vel [geometry_msgs/Twist] 1 subscriber
~~~

7. 查看行为节点图：

~~~shell
rqt_graph
~~~
显式如下节点图：

{% asset_img 节点图.png  %}