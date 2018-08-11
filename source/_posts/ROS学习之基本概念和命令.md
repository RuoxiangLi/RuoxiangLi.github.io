---
title: ROS学习之基本概念和命令
date: 2018-03-22 13:27:48
tags:
  - ROS
categories: ROS
---

-----

这篇文章是有关ROS中基本概念和命令的学习内容。

<!--more--->

# ROS命令**

`source /devel/setup.bash`：刷新`setup.bash`文件，这个自动生成的脚本文件设置了若干环境变量，从而使ROS能够找到创建的功能包和新生成的可执行文件，类似与下面所述的全局脚本文件，但该文件是专门为自己的工作区量身定做的。

`source /opt/ros/kinetic/setup.bash`：全局的脚本文件

`ros··· -h`：查看ros命令格式

`roscore`：运行所有ROS程序前首先要运行的命令

`roswtf`：进行全面深入的检测，包括检测环境变量、安装的文件以及运行的节点

`rosnode list`：列出活跃的节点

`rosnode info /[node_name]`：返回关于一个特定节点的信息

`rosrun [package_name] [node_name]`：允许使用包名直接运行一个包内的节点

`rosrun rqt_graph rqt_graph`：以图的形式显示正在运行的节点和话题之间的消息

`roscp [package_name] [file_to_copy_path] [copy_path]`：将文件从一个package复制到另一个package

`rospack`：允许用户获取软件包的有关信息，用法：`rospack find [package_name]`

`roscd`：切换工作目录到某个软件包或软件包集中

`rosls`：直接按软件包的名称而不是绝对路径执行ls命令，罗列命令。

`catkin_create_pkg <package_name> [depend1] [depend2] [depend3]`：创建程序包

`rospack depends1 <package_name>`：一级依赖

`rospcak depends <package_name>`：间接依赖

`catkin_make [make_targets] [-DCMAKE_VARIABLES=...]`：编译程序包

`rosnode list`：列出正在运行的活跃的节点

`rosnode info [node_name]`：返回关于一个特定节点的信息

`rosnode kill [node_name]`：终止节点运行

`rosnode cleanup`：将节点从rosnode列表中删除

`rosrun [package_name][node_name]`：允许使用包名直接运行一个包内的节点

`rostopic echo [topic]`：显示在某个话题上发布的数据，本质是生成rostopic echo节点，订阅该话题以接收该话题上发布的消息并显示。

`rostopic hz [topic]`：订阅指定的话题，显示该话题的数据发布速率，每秒发布的消息数量

`rostopic bw [topic]`：订阅指定的话题，显示话题使用的宽带，即每秒发布消息所占的字节量

`rostopic list`：列出所有当前订阅和发布的话题，运行`rostopic list -h`可查看其子命令。

`rostopic pub -r rate-in-hz [topic]　[msg_type] [args]`：向当前某个正在广播的话题重复地按照指定频率发布指定的消息，使用`rostopic pub -h`查看该命令参数，例子：

`rostopic pub –r 1 /turtle1/cmd_vel geometry_msgs/Twist ’[2,0,0]’ ’[0,0,0]’`

子参数说明：

 * -r：指定话题以频率模式发布消息，即以一定的时间周期发布消息
 * -1(数字1)：一次性发布模式
 * -l(小写L)：默认的模式，即特别的锁存模式，也是发布一次消息，但会确保该话题的新订阅者也会收到消息
 * -f：从文件中读取消息或从标准的输入中读取

`rostopic type [topic]`：显示所发布话题的消息类型

`rostopic info [topic]`：获取关于话题的信息（消息类型、发布者、订阅者）

`rosmsg show [message type]`：查看消息的详细情况，即消息类型的基本数据类型组成

`rosmsg users`：Find files that use message  

`rosmsg md5`：Display message md5sum  

`rosmsg package`：List messages in a package 

`rosmsg packages`：List packages that contain messages



rosservice命令可以使用ROS客户端/服务器框架提供的服务，其子命令如下：

`rosservice list`：输出可用服务的信息

`rosservice call [service`] [args]：调用带参数的服务

`rosservice type [service]`：输出服务类型

`rosservice find`：依据类型寻找服务

`rosservice uri`：输出服务的`ROSRPC uri`

`rosparam set [param_name]`：设置参数

`rosparam get [param_name]`：获取参数

`rosparam load`：从文件读取参数

`rosparam dump`：向文件中写入参数

`rosparam delete`：删除参数

`rosparam list`：列出参数名

`rosdep install [package]`：下载并安装ROS package所需要的系统依赖项

 

- Roslaunch xml文件标签说明：<http://wiki.ros.org/roslaunch/XML>
- Urdf xml 文件标签说明：<http://wiki.ros.org/urdf/XML>
- Roscpp api 文档：<http://docs.ros.org/jade/api/roscpp/html/>
- Rospy api 文档：http://docs.ros.org/jade/api/rospy/html/

# **ROS概念**

## **ROS文件系统**

文件系统层概念主要指在硬盘里能看到的ROS目录和文件，包括：

- Packages：软件包，ROS应用程序代码的组织单元，每个软件包都可包含程序库、可执行文件、脚本或其他手动创建的文件。
- Manifest（package.xml）：清单是对于“软件包”相关信息的描述，用于定义软件包相关元信息之间的依赖关系，这些信息包括版本、维护者和许可协议等。
- Message (msg) types: 存储在`my_package/msg/MyMessageType.msg`的Message文件，主要定义了ROS系统的messages传输的数据结构。 
- Service (srv) types: 存储在 `my_package/srv/MyServiceType.srv`的服务services文件，定义了ROS的服务通信时的请求（request ）和响应（response ）相关的数据结构。 

## ROS计算图层

计算图是ROS在点对点网络里整合并处理数据的过程。基本计算图概念是 *节点*, *主机*, *参数服务器*, *消息*, *服务*, *话题*, and *数据包*，它们通过不同的方式提供数据给图层。 这些概念是在ros_comm库里实现的。

- **Nodes**: 节点主要执行计算处理 。ROS被设计为细粒度的模块化的系统：一个机器人控制系统通常有很多节点组成 。例如，一个节点控制激光测距仪，一个节点控制轮电机，一个节点执行定位，一个节点执行路径规划，一个节点提供系统图形界面，等等。一个ROS节点通过ROS客户端库 [client library](http://wiki.ros.org/Client%20Libraries)编写，例如 [roscpp](http://wiki.ros.org/roscpp) o或[rospy](http://wiki.ros.org/rospy) 。
- **Master**: The ROS Master provides name registration and lookup to the rest of the Computation Graph. Without the Master, nodes would not be able to find each other, exchange messages, or invoke services.  
- **Parameter Server**: The Parameter Server allows data to be stored by key in a central location. It is currently part of the Master. 
- **Messages**: 节点之间使用messages信息互相通信。 一个消息就是一个由类型域组成的简单的数据结构，支持标准的原始数据类型（integer, floating point, boolean等等）和数组 。消息可以包含任意嵌套的结构和数组（很像C结构）。
- **Topics**: Messages are routed via a transport system with publish / subscribe semantics.  A node sends out a message by *publishing* it to a given [topic](http://wiki.ros.org/Topics). The topic is a [name](http://wiki.ros.org/Names) that is used to identify the content of the message.  A node that is interested in a certain kind of data will *subscribe* to the appropriate topic.  There may be multiple concurrent publishers and subscribers for a single topic, and a single node may publish and/or subscribe to multiple topics.  In general, publishers and subscribers are not aware of each others' existence.  The idea is to decouple the production of information from its consumption. Logically, one can think of a topic as a strongly typed message bus.  Each bus has a name, and anyone can connect to the bus to send or receive messages as long as they are the right type. 
- **Services**: The publish / subscribe model is a very flexible communication paradigm, but its many-to-many, one-way transport is not appropriate for request / reply interactions, which are often required in a distributed system.  Request / reply is done via [services](http://wiki.ros.org/Services), which are defined by a pair of message structures: one for the request and one for the reply. A providing node offers a service under a [name](http://wiki.ros.org/Names) and a client uses the service by sending the request message and awaiting the reply.  ROS client libraries generally present this interaction to the programmer as if it were a remote procedure call. 
- **Bags**: Bags are a format for saving and playing back ROS message data. Bags are an important mechanism for storing data, such as sensor data, that can be difficult to collect but is necessary for developing and testing algorithms. 

## 工作空间结构

- build：build space默认的所在位置，同时也是cmake 和 make被调用来配置并编译程序包的地方
- devel：devel space默认的所在位置，也是安装程序包之前存放可执行文件和库文件的地方
- src：存放软件包的位置

## **文件系统工具**

`rospack`：允许用户获取软件包的有关信息，用法：`rospack find [package_name] `

`roscd`：是rosbash命令集中的一部分，允许用户切换工作目录到某个软件包或软件包集中；和ROS中其他工具一样，只能切换到那些路径已经包含在`ROS_PACKAGE_PATH`环境变量中的软件包，可以使用`echo $ROS_PACKAGE_PATH`查看其中包含的路径。`ROS_PACKAGE_PATH`环境变量应该包含那些保存有ROS软件包的路径，并且每个路径之间用冒号分隔开。

`rosls`：rosbash命令集中的一部分，允许用户直接按软件包的名称而不是绝对路径执行`ls`命令，罗列命令。

## **ROS catkin程序包**

组成：`package.xml`文件+`CMakeLists.txt`文件。

每个目录下只能有一个程序包，同一目录下不能有嵌套的或多个程序包存在。

## **创建程序包的命令**

`catkin_create_pkg <package_name> [depend1] [depend2] [depend3]`

该命令需要在工作空间/src目录下执行，`<package_name>`是要创建的软件包的名字，`depend1..3`是创建的程序包依赖的其他程序包,执行完该命令后,就会在src目录下生成一个文件夹，包含`package.xml`和`CMakeLists.txt`文件。

程序包依赖关系查看命令:

一级依赖：`rospack depends1 <package_name>`

间接依赖：`rospcak depends <package_name>`

## **编译程序包**

`catkin_make [make_targets] [-DCMAKE_VARIABLES=...]`

`catkin_make install ` # (可选)

编译工作空间下的某个软件包：

~~~shell
catkin_make  -DCATKIN_WHITELIST_PACKAGES="package1;package2"
~~~



在工作空间下执行上述命令，会编译src文件夹下的所有catkin工程。

## **ROS图概念**

Nodes：节点，ROS网络中的可执行文件，可通过ROS客户库与其他节点通信,节点可以发布或接收一个话题，节点也可以提供或使用某种服务。

Messages：消息，一种ROS数据类型，用于订阅或发布到一个话题

Topics：话题，节点可以发布消息到话题，也可以订阅话题以接受消息

Master：节点管理器，ROS名称服务（如帮助节点找到彼此）

ROS客户端库允许使用不同编程语言编写的节点之间互相通信：

rospy = python客户端

roscpp = c++ 客户端

 

`rosout`：ROS中相当于stdout/stderr，用于收集和记录节点调试输出信息，它总是运行的。

`roscore`：主机+rosout+参数服务器；运行所有ROS程序前首先要运行的命令;启动节点管理器（The Master）

`rosnode list`：列出活跃的节点

`rosnode info /[node_name]`：返回关于一个特定节点的信息

`rosrun [package_name] [node_name]`：允许使用包名直接运行一个包内的节点（不需要知道包的路径）

## **ROS话题**(topic)

节点和节点之间是通过一个ROS话题来互相通信的，某一个节点在一个话题上发布特定的消息，其他节点可以订阅该话题以接收该消息。

rostopic命令工具可以获取有关ROS话题的信息，运行rostopic -h可以查看所有的rostopic子命令。

`rostopic bw`：显示话题使用的宽带。

`rostopic echo [topic]`：显示在某个话题上发布的数据，本质是生成rostopic echo节点，订阅该话题以接收该话题上发布的消息并显示。

`rostopic hz [topic]`：显示话题的数据发布速率

`rostopic list`：列出所有当前订阅和发布的话题，运行rostopic list -h可查看其子命令。

`rostopic pub [topic] [msg_type] [args]`：向当前某个正在广播的话题发布数据，使用`rostopic pub -h`查看该命令参数

`rostopic type [topic]`：显示所发布话题的消息类型，可以根据显示的话题类型，再继续执行`rosmsg show [message type]`：查看消息的详细情况

## **ROS消息(msg)**

话题之间的通信是通过节点之间发送ROS消息实现的，发布器和订阅器之间的通信，必须发送和接收相同类型的消息，意味着话题的类型是由发布在它上面的消息类型决定的。

msg文件存放在package的msg目录下，它是一个描述ROS中所使用消息类型的简单文本，实际是每行声明一个数据类型和变量名，会被用于生成不同语言的源代码。

`rostopic type [topic]`：用来显示所发布话题的消息类型

`rosmsg show [message type]`：查看消息的详细情况，即消息类型的基本数据类型组成

`rosmsg users`：Find files that use message  

`rosmsg md5`：Display message md5sum  

`rosmsg package`：List messages in a package  

`rosmsg packages`：List packages that contain messages

## **ROS服务(srv)**

服务（services）是节点之间通信的另一种方式，服务允许节点发送请求（request）并获得一个响应（response）。srv文件存放在srv目录下，一个srv文件描述一项服务，包含请求和响应两个部分，在srv文件中由'---'分隔。

 rosservice命令可以使用ROS客户端/服务器框架提供的服务，其子命令如下：

`rosservice list`：输出可用服务的信息

`rosservice call [service] [args]`：调用带参数的服务

`rosservice type [service]`：输出服务类型

`rosservice find`：依据类型寻找服务

`rosservice uri`：输出服务的ROSRPC uri

## **ROS参数**

`rosparam`使得能够存储并操作ROS参数服务器（Parameter Server）上的数据，参数服务器能够存储整型、浮点、布尔、字符串、字典和列表等数据类型，使用YAML标记语言的语法，其子命令如下：

`rosparam set [param_name]`：设置参数

`rosparam get [param_name]`：获取参数

`rosparam load`：从文件读取参数

`rosparam dump`：向文件中写入参数

`rosparam delete`：删除参数

`rosparam list`：列出参数名

## **消息发布器（节点）**

创建过程：

- 初始化 ROS 系统 
- 在 ROS 网络内广播将要在话题上发布的某一类型的消息 
- 以某一频率在话题上发布消息 



## **消息订阅器（节点）**

创建过程：

- 初始化ROS系统
- 订阅话题
- 进入自循环，等待消息的到达
- 当消息到达，调用回调函数 

## **服务器（Service）节点、客户端（Client）节点**

## **录制与回放数据**

只有消息已经发布了才可以被录制。

使用`rostopic list -v`命令查看当前系统中发布的所有话题。

在一个保存录制的目录下运行`rosbag record -a`命令，附加的-a选项表示将当前发布的所有话题数据都录制保存到一个bag文件中，该文件会自动以年份、日期和时间命名并以.bag作为后缀，它包含了rosbag record运行期间所有节点发布的话题。

bag文件可以使用`rosbag info`检查其内容，使用`rosbag play`命令回放出来。

 

## **常见错误**

执行`roscd`命令时，出现no such packag的情况，解决方案执行：

~~~yaml
echo "export ROS_PACKAGE_PATH"=~/catkin_ws:"$ROS_PACKAGE_PATH " >> ~/.bashrc
~~~

在新的终端执行：`roscd ...`成功。

 