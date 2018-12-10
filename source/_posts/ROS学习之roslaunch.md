---
title: ROS学习之roslaunch
date: 2018-05-24 20:43:34
tags:
  - roslaunch
categories: 
  - 机器人
  - ROS
copyright: true
---

-----

这篇文章是有关ROS中roslaunch使用的学习内容。

<!--more--->

### wiki官方文档

roslaunch命令：http://wiki.ros.org/roslaunch/Commandline%20Tools

launch文件格式：http://wiki.ros.org/roslaunch/XML

### 介绍

rosrun只能运行一个node，roslaunch可以同时运行多个nodes。roslaunch工具是ros中python实现的程序启动工具，可以通过读取启动文件（launch file）中的参数配置、属性配置等，同时启动节点管理器（master）和多个节点，在启动任何一个节点前，`roslaunch` 将会确定 **roscore节点（节点管理器）** 是否已经在运行，如果没有，自动启动它；可以在本地或者远程（使用SSH）启动ROS节点，要通过参数服务器设置参数。

`roscore`会做三件事： 

- 启动master节点，该节点是隐藏的，用于通过消息名查询目标节点，实现消息、服务在各个节点之间的连接 
- 启动参数服务器parameter server，用于设置与查询参数 
- 启动日志节点，记录所有消息收发和stdout、stderr，目前roscore暂不会加入其他功能

|     任务名称     |                          任务功能                          | 特性 |
| :--------------: | :--------------------------------------------------------: | :--: |
|      master      | 通过消息名查询目标节点，实现消息、服务在各个节点之间的连接 | 隐藏 |
| parameter server |                       设置与查询参数                       |  -   |
|     日志节点     |              记录所有消息收发和stdout、stderr              |  -   |

任何包含两个或两个以上节点的系统都可以利用启动文件来指定和配置需要使用的节点。通常的命名方案是以*.launch*作为启动文件的后缀，启动文件是XML文件。一般把启动文件存储在取名为launch的目录中。

roslaunch命令执行launch文件的命令格式：

~~~shell
roslaunch [package] [filename.launch]
~~~

###  launch文件编写

一般格式：

~~~xml
<launch>
    <node .../>
    <rosparam ..../>
    <param .../>
    <include .../>
    <env .../>
    <remap .../>
    <arg .../>
    <group> 
        ... 
    </group>
</launch>
~~~

- `<launch>...</launch>`：根元素，作为放置其他元素的容器，其他元素必须在该标记之间。

- `<node .../>`：标记用于定义一个希望启动的ROS节点。格式：

  ```xml
  <node name="bar1" pkg="foo_pkg" type="bar" />
  ```

  > 三个必须的属性：pkg, type, name。
  >
  > - pkg是节点所在的程序包名字；
  > - type是节点的类型，是可执行文件的名字；
  > - name是节点名字，不能包含namespace，可以任意给出的，它覆盖了原有文件中ros::init指定的node name；
  > - 在默认状态下，从启动文件启动节点的标准输出被重定向到一个日志文件中，而不是像 `rosrun` 命令那样，将 **log** 信息显示在终端(**console**)。该日志文件的名称是：` ~/.ros/log/run_id/node_name-number-stout.log` 其中，run_id 是节点管理器（master）启动时生成的一个唯一标示符；
  > - 如果需要将标准输出信息输出到终端，使用属性`output`，即`output=screen`；
  > - 其他属性：args（可以通过命令行启动参数赋值，将参数传递给节点）、ns（节点定义为某个namespace下）等。

- `<param .../>`：定义一个设置在参数服务器中的参数，该标记可以放在`<node .../>`标记内部，作为私有参数。格式：

  ~~~xml
  <param name="publish_frequency" type="double" value="10.0" />
  ~~~

  name是参数名，可以给出参数所在namesapce；value是可选属性，用于定义参数值，省略时要通过其他方式（命令行或者binfile、testfile文件）指定参数值；type定义参数的数据类型，也是可选属性，省略时roslaunch会尝试自动定义参数的类型（根据参数的形式进行判断）。

- `<include .../>`：允许当前launch文件包含（调用）其他launch文件，包括该文件中的所有nodes和parameters。格式：

  ~~~xml
  <include file=”$(find package_name)/launch_file_name”/>
  ~~~

  需要写出该launch文件的绝对路径，比较繁琐，一般使用上述方式。roslaunch会搜索package下的所有子目录；因此，必须给出package_name。另外，include也支持ns属性，将它的内容放进指定的namespace，格式：

  ~~~xml
  <include file=”...” ns=”namespace_name”/>
  ~~~

- `<arg .../>`：启动参数，是局部的，只能在一个launch文件中使用，类似于局部变量，声明格式：

  ~~~xml
  <arg name=”arg_name”>
  ~~~

  launch文件中的每个argument必须有指定值。指定argument的值有多种方式，包括命令行赋值、声明argument时赋值。

  - 还支持启动参数，有时也简称为参数甚至args。

  - 命令行赋值。命令格式：

    ```shell
    roslaunch package_name launch_file_name arg_name:=arg_value
    ```

  - 声明时赋值（两种形式）。格式：

    ```xml
    <arg name=”arg_name” default=”arg_name”/>
    <arg name=”arg_name” value=”arg_name”/>
    ```

    两种形式的区别在于，命令行参数可以覆盖default，但是不能重写value的值。

  可以通过arg获取变量的值，格式：

  ~~~xml
  $(arg arg_name)
  ~~~

  另外，还可以将变量值传给included launch文件，格式：

  ~~~xml
  <include file=”path-to-file”>
      <arg name=”arg_name” value=”arg_value”/>
      ...
  </include>
  ~~~

  若在launch文件中，launch文件及其包含的launch文件出现出现相同的arguments，则需在launch文件及included launch文件中同时写：

  ~~~xml
  <arg name=”arg_name” value=”$(arg arg_name)”/>
  ~~~

  第一个arg_name表示included launch文件中的argument，第二个arg_name表示launch文件中的argument。其结果是指定的argument在launch文件及included launch文件中都有相同的值。

  在ROS中prarmeter和argument是不同的，虽然翻译一样。parameter是运行中的ROS系统使用的数值，存储在参数服务器（parameter server）中，每个活跃的节点都可以通过 ros::param::get 函数来获取parameter的值，用户也可以通过rosparam来获得parameter的值。而argument只在启动文件内才有意义，它们的值是不能被节点直接获取的。

- `<remap .../>`：重映射。重映射是基于替换的思想，每个重映射包含一个原始名称和一个新名称。每当节点使用重映射中的原始名称时，ROS客户端库就会将它默默地替换成其对应的新名称。例如，运行一个 turtlesim 的实例，如果想要把海龟的姿态数据发布到话题/tim 而不是/turtle1/pose，就可以使用如下命令：

  ~~~shell
  rosrun turtlesim turtlesim_node turtle1/pose:=tim 
  ~~~

  通过启动文件的方式，只需在启动文件内使用重映射（remap）元素即可：

  ~~~xml
  <remap from=”turtle1/pose” to ”tim”/>
  ~~~

  例如，节点`mono`订阅了`/camera/image_raw`话题，但是现在只有`/camera_node/image_raw`话题在发布和`/camera/image_raw`话题一样的数据，可以使用重映射完成数据的订阅，这样就可以使得节点`mono`能够订阅`/camera_node/image_raw`话题的数据：

  ~~~xml
  <remap from="/camera/image_raw" to="/camera_node/image_raw"/>
  ~~~

- `<group> ... </group>`：可以将指定的nodes组织起来，只能使用ns、if、unless三个属性。group有两个作用/好处：

  - 可以将几个nodes放进同一个namespace，从而使该组标签有独立的名称空间，格式：

    ~~~xml
    <group ns=”namespace”>
        <node pkg=”..” .../>
        <node pkg=”..” .../>
        ......
    </group>
    ~~~

    如果grouped node已经有它自己的namespace，并且是relative name，那么该node的namespace是其relative name，并以group namespace为后缀。

  - 可以同时启动或者终止一组nodes，格式：

    ~~~xml
    <group if=”$(arg arg_name)”>
    	......
    </group>
    <group unless=”$(arg arg_name)”>
    	......
    </group>
    ~~~

    其中arg_name的值只有0或1，若真，则包含group标签，其中的nodes都可运行；否则其中的nodes都不会运行。​

  ​


参考链接：

https://blog.csdn.net/fengmengdan/article/details/42984429（启动文件的编写）

http://www.cnblogs.com/zjiaxing/p/5542614.html（启动文件的编写）

https://www.cnblogs.com/zjiaxing/p/5541841.html（ros命名空间的解释）
