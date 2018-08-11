---
title: ROS学习之actionlib库（４）-实践之小乌龟画五边形
date: 2018-03-31 11:19:47
tags:
  - ROS actionlib
categories: ROS
---

-----

这篇文章是有关ROS中actionlib使用的学习内容。

<!--more-->

本例程并没有创建`.action`文件生成消息，在安装ROS时，在`opt/ros/kinetic/`路径中已经包含了我们需要的文件，其实就是一个依赖库`turtle_actionlib`。这一点不同就需要在`CMakeList.txt`和`package.xml`文件中手动添加一些信息，后续的内容会提到。

# 创建行为服务器

创建文件`actionlib_tutorials/src/shape_server.cpp`：

```c++
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <actionlib/server/simple_action_server.h>
#include <cmath>
#include <math.h>
#include <angles/angles.h>

#include <geometry_msgs/Twist.h>
#include <turtle_actionlib/ShapeAction.h>

// This class computes the command_velocities of the turtle to draw regular polygons 
class ShapeAction
{
public:
  ShapeAction(std::string name) : 
    as_(nh_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&ShapeAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&ShapeAction::preemptCB, this));

    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/turtle1/pose", 1, &ShapeAction::controlCB, this);
    pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);

    as_.start();
  }

  ~ShapeAction(void)
  {
  }

  void goalCB()
  {
    // accept the new goal
    turtle_actionlib::ShapeGoal goal = *as_.acceptNewGoal();
    //save the goal as private variables
    edges_ = goal.edges;
    radius_ = goal.radius;

    // reset helper variables
    interior_angle_ = ((edges_-2)*M_PI)/edges_;
    apothem_ = radius_*cos(M_PI/edges_);
    //compute the side length of the polygon
    side_len_ = apothem_ * 2* tan( M_PI/edges_);
    //store the result values
    result_.apothem = apothem_;
    result_.interior_angle = interior_angle_;
    edge_progress_ =0;
    start_edge_ = true;
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void controlCB(const turtlesim::Pose::ConstPtr& msg)
  {
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
  
    if (edge_progress_ < edges_)
    {
      // scalar values for drive the turtle faster and straighter
      double l_scale = 6.0;
      double a_scale = 6.0;
      double error_tol = 0.00001;

      if (start_edge_)
      {
        start_x_ = msg->x;
        start_y_ = msg->y;
        start_theta_ = msg->theta;
        start_edge_ = false;
      }

      // compute the distance and theta error for the shape
      dis_error_ = side_len_ - fabs(sqrt((start_x_- msg->x)*(start_x_-msg->x) + (start_y_-msg->y)*(start_y_-msg->y)));
      theta_error_ = angles::normalize_angle_positive(M_PI - interior_angle_ - (msg->theta - start_theta_));
     
      if (dis_error_ > error_tol)
      {
        command_.linear.x = l_scale*dis_error_;
        command_.angular.z = 0;
      }
      else if (dis_error_ < error_tol && fabs(theta_error_)> error_tol)
      { 
        command_.linear.x = 0;
        command_.angular.z = a_scale*theta_error_;
      }
      else if (dis_error_ < error_tol && fabs(theta_error_)< error_tol)
      {
        command_.linear.x = 0;
        command_.angular.z = 0;
        start_edge_ = true;
        edge_progress_++;
      }  
      else
      {
        command_.linear.x = l_scale*dis_error_;
        command_.angular.z = a_scale*theta_error_;
      } 
      // publish the velocity command
      pub_.publish(command_);
      
    } 
    else
    {          
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }   
  }

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<turtle_actionlib::ShapeAction> as_;
  std::string action_name_;
  double radius_, apothem_, interior_angle_, side_len_;
  double start_x_, start_y_, start_theta_;
  double dis_error_, theta_error_;
  int edges_ , edge_progress_;
  bool start_edge_;
  geometry_msgs::Twist command_;
  turtle_actionlib::ShapeResult result_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_shape");

  ShapeAction shape(ros::this_node::getName());
  ros::spin();

  return 0;
}
```

# 创建行为客户端

创建行为客户端文件`actionlib_tutorials/src/shape_client.cpp`：

```c++
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <turtle_actionlib/ShapeAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_shape"); 

  // create the action client
  // true causes the client to spin it's own thread
  actionlib::SimpleActionClient<turtle_actionlib::ShapeAction> ac("turtle_shape", true); 

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
 
  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action 
  turtle_actionlib::ShapeGoal goal;
  goal.edges = 5;
  goal.radius = 1.3;
  ac.sendGoal(goal);
  
  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(40.0));

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

**注意**：

本例程并没有创建`.action`文件生成消息，而是使用安装ROS时就有的`opt/ros/kinetic/share/turtle_actionlib`目录下的库文件，所以在`CMakeLists.txt`文件中需要添加如下信息：

~~~cmake
catkin_package(
   CATKIN_DEPENDS actionlib_msgs turtle_actionlib #turtle_actionlib是新添加的依赖库
)
~~~

在`CMakeLists.txt`文件末尾添加以下几行： 

```cmake
add_executable(shape_server src/shape_server.cpp)
target_link_libraries(shape_server ${catkin_LIBRARIES})

add_executable(shape_client src/shape_client.cpp)
target_link_libraries(shape_client ${catkin_LIBRARIES})
```

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
   CATKIN_DEPENDS actionlib_msgs turtle_actionlib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fibonacci_server src/fibonacci_server.cpp)
target_link_libraries(fibonacci_server ${catkin_LIBRARIES})

add_executable(fibonacci_client src/fibonacci_client.cpp)
target_link_libraries(fibonacci_client ${catkin_LIBRARIES})

add_executable(averaging_server src/averaging_server.cpp)
target_link_libraries(averaging_server ${catkin_LIBRARIES})

add_executable(averaging_client src/averaging_client.cpp)
target_link_libraries(averaging_client ${catkin_LIBRARIES})

add_executable(do_dishes_server src/do_dishes_server.cpp)
target_link_libraries(do_dishes_server ${catkin_LIBRARIES})

add_executable(do_dishes_client src/do_dishes_client.cpp)
target_link_libraries(do_dishes_client ${catkin_LIBRARIES})

add_executable(shape_server src/shape_server.cpp)
target_link_libraries(shape_server ${catkin_LIBRARIES})

add_executable(shape_client src/shape_client.cpp)
target_link_libraries(shape_client ${catkin_LIBRARIES})
```

接着记得在`package.xml`文件中加入如下信息：

~~~xml
<build_depend>turtle_actionlib</build_depend><build_export_depend>turtle_actionlib</build_export_depend><exec_depend>turtle_actionlib</exec_depend>
~~~

工作空间下执行`catkin_make`命令。

# 运行行为

终端启动ROS：

```shell
roscore
```

运行小乌龟：

~~~
rosrun turtlesim turtlesim_node 
~~~

运行行为客户端：

```shell
rosrun actionlib_tutorials shape_client 
```

运行行为服务器：

```shell
rosrun actionlib_tutorials shape_server
```
执行命令查看反馈信息：

~~~
rostopic echo /turtle_shape/feedback
~~~

程序执行完并没有反馈信息输出，因为没有涉及到反馈信息。

执行命令查看结果信息：

~~~
rostopic echo /turtle_shape/result
~~~

小乌龟画完五边形，会有信息输出：

~~~
header: 
  seq: 0
  stamp: 
    secs: 1522547343
    nsecs: 139139044
  frame_id: ''
status: 
  goal_id: 
    stamp: 
      secs: 1522547324
      nsecs: 173274728
    id: "/test_shape-1-1522547324.173274728"
  status: 3
  text: ''
result: 
  interior_angle: 1.88495564461
  apothem: 1.05172204971
---
~~~

小乌龟画图的效果：

{% asset_img 五边形.png  %}

执行命令`rqt_graph`查看节点图如下所示：

{% asset_img 节点图.png  %}

理解分析：

服务器是作为ROS中的一个节点存在的，该节点名称为`/turtle_shape`，它会发布消息到话题`/turtle_shape/feedback`、　`/turtle_shape/result`、`/turtle_shape/status`，客户端通过订阅这些话题获取到服务器执行客户端赋予的任务的完成进度信息；服务器还发布消息到`/turtle1/cmd_vel`话题，`/turtlesim`订阅了该话题，以此来控制小乌龟运动。同时，服务器订阅了`turtle_shape/goal`、`/turtle_shape/cancel`、`/turtle/pose`三个话题，服务器通过话题`turtle_shape/goal`获取到客户端发布的目标信息，通过`turtle_shape/cancel`话题获取到客户端发布的中断消息。

客户端也是一个节点`test_shape`，该节点订阅了`/turtle_shape/feedback`、　`/turtle_shape/result`、`/turtle_shape/status`三个话题，并发布消息到`/turtle_shape/goal`和`/turtle_shape/cancel`话题，前者使的服务器获取到客户端发布的任务目标，后者是客户端告知服务器中断任务停止执行的途径。

根据上面的节点图，笔者总结了一下actionlib SimpleAction的简单交互图，帮助自己理解。笔者理解的是`feedback`、`result`、`status`、`goal`、`cancel`，这些话题由于actionlib机制的存在会自动创建，并且ActionServer和ActionClient会自动发布或者订阅这些话题，这也许就是actionlib的作用了。仔细想想，其实和简单的消息发布器、接收器的核心思想是一样的。（这一点理解在官网的介绍中得到验证，“action客户端和服务端通过预定义的ROS Action协议通信，该通信机制基于ROS消息”）

{% asset_img 交互图.png  %}

如果要画其他多边形，将客户端文件中的`goal`对象的`edges`值设为相应的边数即可。