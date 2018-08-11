---
title: ROS学习之编写简单的服务器和客户端
date: 2018-03-28 22:11:20
tags: 
  - C++
  - catkin
  - ROS服务器
  - ROS客户端
categories: ROS
---

-----

这篇文章是有关ROS中服务器和客户端的学习内容。

<!--more--->

**写在篇头：**

ROS程序包中一般包含`msg`、`src`、`srv`、`scripts`目录，分别存放`msg`消息文件、C++源文件（`.cpp`）、`srv`服务文件、Python源文件（`.py`），执行`catkin_make`命令编译完成后，`.msg`文件、`srv`文件都会转换为ROS所支持的源代码，并生成C++可执行文件。

1. C++

   - `.msg`、`.srv`文件生成的C++头文件将放在`~/工作空间/devel/include/程序包名/`下
   - 生成的可执行文件放在`~/工作空间/devel/lib/<package name>` 下

2. Python

   - `.msg`文件生成的Python`.py`脚本文件放在 `~/工作空间/devel/lib/python2.7/dist-packages/程序包名/msg`下

   - `.srv`文件生成的Python`.py`脚本文件放在 `~/工作空间/devel/lib/python2.7/dist-packages/程序包名/srv`下

   - 使用`chmod +x scripts/xxx.py`命令，使节点文件具有执行属性

     <!--more--->

# 消息(msg)和服务(srv)介绍

- 消息(msg): msg文件就是一个描述ROS中所使用消息类型的简单文本，被存放在package的msg目录下。该文件会被用来生成不同语言的源代码，一般是C++、Python。 msg文件实际上就是每行声明一个数据类型和变量名，可以使用的数据类型如下： 
  - int8, int16, int32, int64 (plus uint*) 
  - float32, float64 
  - string 
  - time, duration 
  - other msg files 
  - variable-length array[] and fixed-length array[C] 


- 服务(srv): 一个srv文件描述一项服务， srv文件被存放在srv目录下。 srv文件分为请求和响应两部分，由'---'分隔。下面是srv的一个样例： 

  ```
  int64 A
  int64 B
  ---
  int64 Sum
  ```

  其中 `A` 和 `B` 是请求, 而`Sum` 是响应。 

# 创建完成该任务的程序包

```shell
cd ~/catkin_ws/src #工作空间catkin_ws
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp #创建程序包
mkdir -p beginner_tutorials/src　#放置所有源代码
```

# 创建msg

1. 创建`msg`消息

   ~~~shell
   mkdir msg #在新创建的程序包目录下
   echo "int64 num" > msg/Num.msg
   ~~~


2. 配置文件

   - `package.xml`中添加：

     ~~~xml
     <build_depend>message_generation</build_depend
     <run_depend>message_runtime</run_depend>
     ~~~

     ​

   - `CMakeList.txt`添加信息的部分：

     ~~~cmake
     find_package(catkin REQUIRED COMPONENTS
       roscpp
       rospy
       std_msgs
       message_generation
     )
     add_message_files(
       FILES
       Num.msg
     )
     generate_messages(
       DEPENDENCIES
       std_msgs
     )
     catkin_package(
      CATKIN_DEPENDS message_runtime
     )
     ~~~

**注意：**执行`catkin_make`编译程序包后，某个程序包中的`.msg`文件都会转换为ROS所支持的源代码，生成的C++头文件将会放置在`~/工作空间/devel/include/程序包名/`下，本程序生成`Num.h`。Python脚本语言会在 `~/工作空间/devel/lib/python2.7/dist-packages/程序包名/msg` 目录下创建。

# 　创建srv

1. 创建`srv`文件

   ~~~shell
   mkdir msg　#在新创建的程序包目录下
   roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv #从其他程序包中复制一个服务文件
   ~~~


2. 配置文件

   `CMakeList.txt`添加信息的部分：

   ```cmake
   add_service_files(
     FILES
     AddTwoInts.srv
   )
   ```

**注意：**执行`catkin_make`编译程序包后，某个程序包中的`.srv`文件都会转换为ROS所支持的源代码，生成的C++头文件将会放置在`~/工作空间/devel/include/程序包名/`下，这里生成的是`AddTwoInts.h`。Python脚本语言会在 `~/工作空间/devel/lib/python2.7/dist-packages/程序包名/srv` 目录下创建。

# 创建Service节点

创建一个简单的service节点`add_two_ints_server`，该节点将接收到两个整形数字，并返回它们的和。 

在程序包创建`src/add_two_ints_server.cpp`文件：

~~~c++
#include "ros/ros.h"
//编译系统自动根据先前创建的srv文件生成的对应该srv文件的头文件
#include "beginner_tutorials/AddTwoInts.h"

//提供两个int值的求和服务，int值从request中获取，返回数据装入response，这些数据类型都定义在srv文件内部
bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  //service已经建立起来，并在ROS内发布出来，参数add_two_ints与客户端创建client时的参数一致
  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
~~~

# 创建Clinet节点

在程序包创建`src/add_two_ints_client.cpp`文件：

~~~c++
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  //为add_two_ints service创建一个client。ros::ServiceClient 对象会用来调用service
  ros::ServiceClient client = 
      						n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);//long long int atoll ( const char * str );
  srv.request.b = atoll(argv[2]);
  
  //调用service，该过程是模态过程（调用的时候占用进程阻止其他代码的执行），一旦调用完成，将返回调用结果。如果service调用成功，call()函数将返回true，srv.response里面的值将是合法的值。如果调用失败，call()函数将返回false，srv.response里面的值将是非法的。 
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
~~~

# 编译节点

在`CMakeList.txt`文件末尾添加：

```cmake
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
add_dependencies(add_two_ints_server beginner_tutorials_gencpp)　#不加这句也可以

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
add_dependencies(add_two_ints_client beginner_tutorials_gencpp)　#不加这句也可以
```

工作空间下执行命令：

~~~shell
catkin_make
~~~

执行结束将生成两个可执行程序`add_two_ints_server`和`add_two_ints_client`，默认放在`devel space`下的包目录下，即`~/工作空间/devel/lib/<package name>`。可以直接调用可执行程序，或者使用rosrun命令去调用。

# 运行节点

1. 启动ROS

   ```shell
   roscore
   ```

2. 启动服务端

   ```shell
   rosrun beginner_tutorials add_two_ints_server
   ```

3. 启动客户端（带参数）

   ```powershell
   rosrun beginner_tutorials add_two_ints_client 1 3
   ```

这样在服务端终端和客户端终端就会看到相应的输出信息。
