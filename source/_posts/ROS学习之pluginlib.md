---
title: ROS学习之pluginlib
date: 2018-04-03 19:23:25
tags:
  - ROS
categories: 
  - 机器人
  - ROS
copyright: true
---

-----

这篇文章是有关ROS pluginlib使用的学习内容。

<!--more--->

ROS的pluginlib程序包提供了一种使用ROS构建基础结构编写和**动态加载**插件的工具。为了能够工作，这些工具需要**插件提供者在他们的包的package.xml中注册他们的插件。**

# 概述

pluginlib是一个C ++库，用于**从ROS包**中加载和卸载插件。插件是从运行时库（即共享对象，动态链接库）加载的可动态 	加载的类。使用pluginlib，不需要将应用程序明确地链接到包含类的库。相反，pluginlib可以在任何时候打开包含导出类的库，而无需事先知道库或包含类定义的头文件。 插件对于扩展/修改应用程序行为而不需要应用程序源代码是很有用的。

 **pluginlib**利用了**C++多态的特性**，不同的插件只要使用统一的接口（抽象基类）便可以替换使用。这样用户通过调用在插件中实现的统一的接口函数，不需要更改程序，也不需要重新编译，更换插件即可实现功能修正。

利用**pluginlib**编写插件的方法大致包括如下四步：

1. 创建插件基类，定义统一接口（如果为现有接口编写插件，则跳过该步）
2. 编写插件类，继承插件基类，实现统一接口
3. 导出插件，并编译为动态库
4. 将插件加入ROS系统，使其可识别和管理

贴一张自己总结的pluginlib框架图，帮助自己理解：

{% asset_img pluginlib.png  %}

# 例子

首先，假设存在一个包含多边形基类的ROS程序包（“polygon_interface_package”）。系统支持两种不同的多边形：一个是位于“rectangle_plugin”包中的矩形和一个是位于“triangle_plugin”包中的三角形。rectangle_plugin和triangle_plugin包的实现都在其package.xml文件中包含特殊的导出行，告诉rosbuild系统它们可以为polygon_interface_package包中的polygon类提供插件。实际上这些导出行作用是在ROS构建/打包系统中注册这些类。这样使用者如果希望在系统中看到所有的多边形类，它就可以运行一个简单的`rospack`命令查询，得到可以使用的类的清单，在这里是三角形和矩形。



![pluginlib/plugin_model.png](http://wiki.ros.org/pluginlib?action=AttachFile&do=get&target=plugin_model.png)

# 提供一个插件

## 注册/导出插件

一个可以被动态加载的类必须被标记为导出类，可以使用特殊的宏`PLUGINLIB_EXPORT_CLASS`实现这一点。该宏可以放入任何组成插件库的源（.cpp）文件中，但通常放在导出类的.cpp文件的末尾。对于上述例子，可以在`example_pkg`包中创建`class_list.cpp`文件，并编译该文件，加入librectangle库：

~~~c++
#include <pluginlib/class_list_macros.h>
#include <polygon_interface_package/polygon.h>
#include <rectangle_package/rectangle.h>

//Declare the Rectangle as a Polygon class
PLUGINLIB_EXPORT_CLASS(rectangle_namespace::Rectangle, polygon_namespace::Polygon)
~~~

## 插件描述文件

插件描述文件是一个XML文件，它以机器可读的格式存储关于插件的所有信息，包括插件所在的库、插件的名字、插件的类型等等。上述例子的插件描述文件（例如，`rectangle_plugin.xml`）可以写为：

~~~XML
<library path="lib/librectangle">
  <class type="rectangle_namespace::Rectangle" base_class_type="polygon_namespace::Polygon">
  <description>
  This is a rectangle plugin
  </description>
  </class>
</library>
~~~

关于插件描述文件的详细信息，[查看这里](http://wiki.ros.org/pluginlib/PluginDescriptionFile)。

### 为什么需要这个文件

除了代码宏之外，我们还需要这个文件来允许ROS系统自动发现、加载和推理插件。 插件描述文件也包含重要的信息，如插件的描述，它不适合在宏中使用。

## 使用ROS Package System注册插件

为了让pluginlib可以通过所有的ROS包查询系统上的所有可用插件，每个包必须明确指定它导出的插件以及哪些包库包含这些插件。插件提供程序必须在其导出标记块内的package.xml中指向其插件描述文件。 请注意，如果有其他出口，它们都必须放在同一个出口字段中。对于上述例子，相关的内容写为：

~~~xml
<export>
  <polygon_interface_package plugin="${prefix}/rectangle_plugin.xml" />
</export>
~~~

关于导出一个插件的细节学习，[参考这里](http://wiki.ros.org/pluginlib/PluginExport)。

**提醒：**为了使上述导出命令正常工作，提供的包必须直接依赖于包含插件接口的包。 例如，rectangle_plugin的catkin / package.xml中必须包含以下行：

~~~xml
<build_depend>polygon_interface_package</build_depend>
<run_depend>polygon_interface_package</run_depend>
~~~

## 查询ROS包系统中的可用插件

用户可以使用`rospack`命令查看可用的插件，例如：

~~~
rospack plugins --attrib=plugin nav_core
~~~

其中nav_cor为包名，该命令将返回所有从nav_core包中导出的插件。

# 使用一个插件

pluginlib在class_loader.h头文件中提供了ClassLoader类，使得可以快速方便地使用它提供的类。关于该类细节的描述[参考这里](http://docs.ros.org/api/pluginlib/html/classpluginlib_1_1ClassLoaderBase.html)。下面是一个简单的例子，使用ClassLoader类在使用多边形的代码中创建一个rectangle的实例：

~~~c++
#include <pluginlib/class_loader.h>
#include <polygon_interface_package/polygon.h>

//... some code ...

pluginlib::ClassLoader<polygon_namespace::Polygon> poly_loader("polygon_interface_package", "polygon_namespace::Polygon");

try
{
  boost::shared_ptr<polygon_namespace::Polygon> poly = poly_loader.createInstance("rectangle_namespace::Rectangle");

  //... use the polygon, boost::shared_ptr will automatically delete memory when it goes out of scope
}
catch(pluginlib::PluginlibException& ex)
{
  //handle the class failing to load
  ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
}
~~~

**注意：**在使用插件时，ClassLoader不能超出范围。 所以，如果要在类中加载插件对象，请确保类加载器是该类的成员变量。

# 手动创建并使用一个简单的插件

## 准备工作

安装pluginlib_tutorials pkg：

~~~shell
apt-get install ros-kinetic-common-tutorials
~~~

catkin_ws/src目录下创建程序包：

~~~shell
catkin_create_pkg pluginlib_tutorials_ roscpp pluginlib
~~~

## 创建基类

创建文件`pluginlib_tutorials_/include/pluginlib_tutorials_/polygon_base.h`：

~~~c++
#ifndef PLUGINLIB_TUTORIALS__POLYGON_BASE_H_
#define PLUGINLIB_TUTORIALS__POLYGON_BASE_H_

namespace polygon_base
{
  class RegularPolygon
  {
    public:
      virtual void initialize(double side_length) = 0;
      virtual double area() = 0;
      virtual ~RegularPolygon(){}

    protected:
      RegularPolygon(){}
  };
};
#endif
~~~

## 创建插件

创建文件`include/pluginlib_tutorials_/polygon_plugins.h`：

~~~c++
#ifndef PLUGINLIB_TUTORIALS__POLYGON_PLUGINS_H_
#define PLUGINLIB_TUTORIALS__POLYGON_PLUGINS_H_
#include <pluginlib_tutorials_/polygon_base.h>
#include <cmath>

namespace polygon_plugins
{
  class Triangle : public polygon_base::RegularPolygon
  {
    public:
      Triangle(){}

      void initialize(double side_length)
      {
        side_length_ = side_length;
      }

      double area()
      {
        return 0.5 * side_length_ * getHeight();
      }

      double getHeight()
      {
        return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
      }

    private:
      double side_length_;
  };

  class Square : public polygon_base::RegularPolygon
  {
    public:
      Square(){}

      void initialize(double side_length)
      {
        side_length_ = side_length;
      }

      double area()
      {
        return side_length_ * side_length_;
      }

    private:
      double side_length_;

  };
};
#endif
~~~

## 注册插件

创建文件`src/polygon_plugins.cpp`：

~~~c++
#include <pluginlib/class_list_macros.h>
#include <pluginlib_tutorials_/polygon_base.h>
#include <pluginlib_tutorials_/polygon_plugins.h>

PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
~~~

## 构建插件

`CMakeList.txt`文件中添加内容：

~~~cmake
include_directories(include)
add_library(polygon_plugins src/polygon_plugins.cpp)
~~~

## 使得ROS Toolchain可访问插件

### 创建XML文件

在程序包顶层目录创建文件`polygon_plugins.xml`：

~~~xml
<library path="lib/libpolygon_plugins">
  <class type="polygon_plugins::Triangle" base_class_type="polygon_base::RegularPolygon">
    <description>This is a triangle plugin.</description>
  </class>
  <class type="polygon_plugins::Square" base_class_type="polygon_base::RegularPolygon">
    <description>This is a square plugin.</description>
  </class>
</library>
~~~

### 导入插件

在`package.xml`文件添加信息：

~~~xml
<export>
  <pluginlib_tutorials_ plugin="${prefix}/polygon_plugins.xml" />
</export>
~~~

### 测试插件

执行`catkin_make`命令编译工作空间，并执行如下命令：

~~~shell
rospack plugins --attrib=plugin pluginlib_tutorials_
~~~

输出 `polygon_plugins.xml`文件的路径信息则正确。

## 使用插件

创建文件`src/polygon_loader.cpp`：

~~~c++
#include <pluginlib/class_loader.h>
#include <pluginlib_tutorials_/polygon_base.h>

int main(int argc, char** argv)
{
  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("pluginlib_tutorials_", "polygon_base::RegularPolygon");

  try
  {
    boost::shared_ptr<polygon_base::RegularPolygon> triangle = poly_loader.createInstance("polygon_plugins::Triangle");
    triangle->initialize(10.0);

    boost::shared_ptr<polygon_base::RegularPolygon> square = poly_loader.createInstance("polygon_plugins::Square");
    square->initialize(10.0);

    ROS_INFO("Triangle area: %.2f", triangle->area());
    ROS_INFO("Square area: %.2f", square->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  return 0;
}
~~~

## 运行节点

在 `CMakeLists.txt`文件中添加信息：

~~~cmake
add_executable(polygon_loader src/polygon_loader.cpp)
target_link_libraries(polygon_loader ${catkin_LIBRARIES})
~~~

执行`catkin_make`命令。

### 方式一：命令行rosrun方式

运行可执行文件节点：

~~~shell
rosrun pluginlib_tutorials_ polygon_loader
~~~

输出如下信息：

~~~shell
[ INFO] [WallTime: 1279658450.869089666]: Triangle area: 43.30
[ INFO] [WallTime: 1279658450.869138007]: Square area: 100.00
~~~



### 方式二：启动文件方式

在程序包中创建launch文件夹，并创建文件`polygon_loader.launch`，输入：

~~~~xml
<launch>  
        <node name="polygon_loader" pkg="pluginlib_tutorials_" type="polygon_loader" output="screen"/>  
</launch> 
~~~~

执行命令：`roslaunch pluginlib_tutorials_ polygon_loader.launch`

可以看到同样的输出信息。

启动文件（launch file）方式，是ROS提供的一个同时启动节点管理器（master）和多个节点的途径。任何包含两个或两个以上节点的系统都可以利用启动文件来指定和配置需要使用的节点。通常的命名方案是以*.launch*作为启动文件的后缀，启动文件是XML文件。一般把启动文件存储在取名为launch的目录中。每个XML文件都必须要包含一个根元素。根元素由一对launch标签定义：`<launch> … <launch>`元素都应该包含在这两个标签之内。

节点属性中节点元素的形式：

`<node pkg=”package-name” type=”executable-name” name=”node-name”/>`
