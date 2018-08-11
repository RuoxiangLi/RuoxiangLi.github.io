---
title: ROS学习之actionlib库（１）-actionlib库的介绍
date: 2018-03-29 22:42:46
tags:
  - ROS actionlib
categories: ROS
---

------

这篇文章是有关ROS中actionlib使用的学习内容。

<!--more--->

# 介绍

actionlib软件包为ROS中的可抢占任务提供了一个基于话题的通用接口。在任何一个比较大的基于ROS的系统，都会有这样的情况，向某个节点发送请求执行某一个任务，并返回相应的执行结果，这种请求-响应式的使用场景通常使用ROS提供的服务（services）机制完成。然而，有一些情况服务执行的时间很长，在执行中想要定期获得任务处理的进度，或可能取消执行任务（或请求），例如如果执行各种运动的动作，像控制机械手臂去抓取一个杯子，这个过程可能复杂而漫长，执行过程中还可能强制中断或反馈信息，service机制就无法满足需求，而actionlib就能实现这样的功能。它是ROS中一个很重要的功能包集合（库），可以实现一些简单的状态机功能，算的上是SMACH的一个弱化版。

扩展：

SMACH是一个用于快速创建复杂机器人行为的任务级体系结构。SMACH的核心是独立于ROS的Python库，用于构建分层状态机。SMACH是一个新的库，它利用非常古老的概念来快速创建具有可维护和模块化代码的强大机器人行为。可以使用SMACH建立一个有限状态机，但SMACH能做的更多。SMACH是一个任务级的执行和协调库，并提供集中“状态容器”。一个这样的容器是一个有限状态机，但是这个容器也可以是另一个容器中的状态。更多内容[参考wiki](http://wiki.ros.org/cn/smach)。

# 细节描述

actionlib堆栈提供了一个标准化的接口同可抢占任务进行交互，这方面的例子包括将底座移动到目标位置、执行激光扫描并返回产生的点云、检测门的手柄等等。

## 介绍

下面将描述动作客户端和服务器相互交互的底层机制，如果只是简单的使用actionlib就没必要深入学习了。

## 高级客户端/服务器交互

### 服务器描述

#### 服务器状态机

goal是在ActionClient端启动的（因为client会发送sendgoal嘛），一旦ActionServer接收到goal请求，它就会为这个goal创建一个状态机，来追踪goal的状态转换。注意，状态机跟踪的是goal！而不是不是跟踪ActionServer本身！所以系统中对于每一个goal都会有一个状态机。状态转换图如下所示：

![server_states_detailed.png](http://wiki.ros.org/actionlib/DetailedDescription?action=AttachFile&do=get&target=server_states_detailed.png)

#### 服务器转换状态

这些状态的转换大多是服务的实施者（其实就是服务的程序）触发的，用小一串命令：

- **setAccepted** - 检查到有goal之后，决定开始处理它 
- **setRejected** - 检察到goal后，决定不去处理它，因为它是个无效请求（溢出，资源不可用，无效等） 
- **setSucceeded** - 告知goal被正确执行
- **setAborted** - 告知goal在处理时遇到了问题不得不被终止了
- **setCanceled** - 告知因cancle请求，goal不再被执行了

action client也能异步触发状态转换：

- **CancelRequest**: 客户端通知action server它想要server停止处理这个goal服务端状态

服务端状态

1. 中间状态

   （前面说了，simple的状态有三个，就是等待执行挂起）

   - **Pending** - goal还没有被ActionServer处理
   - **Active** - goal正在被AS处理 
   - **Recalling** - goal没有被处理并且从客户端已发送取消它的命令，但AS还不确定goal已经被取消了（时差导致的？）
   - **Preempting** - goal正被处理呢，从AC端收到了取消请求，但AS还不确定goal已经被取消


2. 终点状态 
   - **Rejected** - AC没有发cancle请求，goal被AS不处理直接拒绝了The goal was rejected by the action server without being processed and without a request from the action client to cancel 
   - **Succeeded** - goal被AS成功实现 was achieved successfully by the action server 
   - **Aborted** - goal被AS终止没有AC的cancle请求
   - **Recalled** - 在AS开始执行之前这个goal被另一个goal或者cancle请求取消了
   - **Preempted** - 处理中的goal被另一个goal或者AC的取消请求给取消了

### 客户端描述

#### 客户端状态机



#### 客户端状态

## Action接口和传输层（协议）

### 数据与

### 信息

#### goal话题

#### cancel话题

#### status话题

#### feedback话题

#### result话题

## 协议

### 简单的行为客户端

#### 客户端状态歧义

#### 多目标策略

#### 线程模型

### 简单的行为服务器

#### 目标通知

#### 线程模型

# Client-Server交互

如下图所示，actionlib的框架实际是一种特殊的客户-服务的模式。除了服务请求的功能外，还可以实时获取服务器执行任务的进度状态，以及强制中断服务的功能。action客户端和服务端通过预定义的ROS Action协议通信，该通信机制基于ROS消息。action客户端和服务端通过函数调用和回调的方式，向用户提供用于请求目标（在客户端发生）或执行目标（服务器端发生）的接口。

![client_server_interaction.png](http://wiki.ros.org/actionlib?action=AttachFile&do=get&target=client_server_interaction.png)

# Action清单：Goal,Feedback,Result

为了使得客户端和服务器之间进行通信，需要定义一些用于二者之间通信的消息，这就是*action清单*。该清单定义客户端和服务器之间通信的Goal、Feedback、Result信息。

- Goal：为了使用action来完成任务，引入可以由ActionClient发送到服务器的Goal概念。对于移动底座的情况，Goal将是PoseStamped消息，其中包含关于机器人应该在世界坐标系移动到何处的信息。对于控制倾斜激光扫描仪的情况，	Goal应该包含扫描参数（最小角、最大角、速度等）。
- Feedback：Feedback为服务器实施者提供了一种方法，告知ActionClient目标的增量变化情况。对于移动底座的情况，它可能是机器人沿着路径运动时当前的姿势；对于控制倾斜激光扫描仪的情况，它可能是扫描完成之前剩下的时间。
- Result：完成目标后，结果会从ActionServer发送到ActionClient。Result同Feedback不同，因为它只发送一次，当行动的目标是提供某种信息时，这一点就非常有用。对于移动底座的情况，Result不是非常重要，但它可以是机器人的最终位姿；对于控制倾斜激光扫描仪的情况，结果可能包含根据请求的扫描生成的点云。

# .action文件

在介绍.action文件之前，先创建一个程序包，用于后续内容的学习，actinlib库相关的文件都放在该程序包下：

```shell
catkin_create_pkg actionlib_tutorials actionlib message_generation roscpp rospy std_msgs actionlib_msgs
```

ROS中使用一个.action文件定义action清单，该文件包含goal、result、feedback的定义，使用---分隔开，它一般会被放置在程序包的action目录下。以洗碟子为例，描述该过程的action清单如下所示：

`actionlib_tutorials/action/DoDishes.action`

~~~
# Define the goal
uint32 dishwasher_id  # Specify which dishwasher we want to use
---
# Define the result
uint32 total_dishes_cleaned
---
# Define a feedback message
float32 percent_complete
~~~

基于`.action`文件会产生６个消息用于客户端和服务器的通信，这一过程在`catkin_make`编译过程自动触发完成。

## Catkin

在当前程序包`CMakeList.txt`文件中`catkin_package()`之前添加：

~~~cmake
add_action_files(
  FILES
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

同时需要在`package.xml`文件中包含如下依赖：

```xml
<build_depend>actionlib</build_depend>
<build_depend>actionlib_msgs</build_depend>
<run_depend>actionlib</run_depend>
<run_depend>actionlib_msgs</run_depend>
```

## Results

执行命令：

```shell
roscd actionlib_tutorials
rosrun actionlib_msgs genaction.py -o msg/ action/Fibonacci.action
```

`genaction.py`文件位于`/opt/ros/kinetic/lib/actionlib_msgs/`目录下。

会出现如下提示信息：

```shell
Generating for action Fibonacci
```
通过手动执行`generation.py`文件，我们就使用DoDishes.action生成了以下消息，并保存在了程序包的`msg/`目录下。这些消息文件将被actionlib内部用于ActionClient和ActionServer之间的通信。

- `DoDishesAction.msg` 
- `DoDishesActionGoal.msg` 
- `DoDishesActionResult.msg` 
- `DoDishesActionFeedback.msg` 
- `DoDishesGoal.msg` 
- `DoDishesResult.msg` 
- `DoDishesFeedback.msg`

**注意**：其实我们可以完全不用手动执行如上操作，手动生成消息文件。我们可以在工作空间目录下执行`catkin_make`命令，就会自动生成的`.msg`和`.h`文件，并分别保存在`工作空间/devel/share/actionlib_tutorials/msg`和`工作空间/devel/include/actionlib_tutorials`目录下。

# ActionClient的使用-C++ SimpleActionClient

以下程序实现了如何将goal发送到名为`do_dishes`的DoDishes ActionServer。创建文件`actionlib_tutorials/src/do_dishes_client.cpp`：

~~~c++
#include <actionlib_tutorials/DoDishesAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<actionlib_tutorials::DoDishesAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "do_dishes_client");
  Client client("do_dishes", true); // true -> don't need ros::spin()
  client.waitForServer();
  actionlib_tutorials::DoDishesGoal goal;
  // Fill in goal here
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The dishes are now clean");
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}
~~~

注意：对于C++SimpleActionClient，在一个单独的线程正在服务客户端的回调队列时，`waitForServer`方法才会工作。这需要传递给客户端构造函数的spin_thread选项，使用多线程微调器运行，或者使用您自己的线程为ROS回调队列提供服务。

# ActionServer的使用-C++ SimpleActionServer

以下片段显示了如何编写一个名为“do_dishes”的DoDishes ActionServer。创建文件`actionlib_tutorials/src/do_dishes_server.cpp`：

~~~c++
#include <actionlib_tutorials/DoDishesAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<actionlib_tutorials::DoDishesAction> Server;

void execute(const actionlib_tutorials::DoDishesGoalConstPtr& goal, Server* as)  // Note: "Action" is not appended to DoDishes here
{
  // Do lots of awesome groundbreaking robot stuff here
  as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "do_dishes_server");
  ros::NodeHandle n;
  Server server(n, "do_dishes", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
~~~

# 测试Action

工作空间目录下执行：

~~~
catkin_make
~~~

执行完会自动生成可执行文件`do_dishes_client`、`do_dishes_server`，保存在`工作空间/devel/lib/actionlib_tutorials`目录下。

终端启动ROS：

```shell
roscore
```

运行行为客户端：

```shell
rosrun actionlib_tutorials do_dishes_client
```

运行行为服务器：

```shell
rosrun actionlib_tutorials do_dishes_server
```

执行完成客户端会有如下输出信息：

~~~
Yay! The dishes are now cleanCurrent State: SUCCEEDED
~~~

执行`rqt_graph`命令查看节点图：

{% asset_img 节点图.png  %}