---
title: ROS学习之catkin CMakeList.txt介绍（译）
date: 2018-03-21 22:04:00
tags: 
  - ROS 
  - CMake
categories: 
  - 机器人
  - ROS
---

***

这篇文章是有关ROS中catkin CMakeLists.txt使用的内容。

<!--more-->

​	本文翻译自ROS官网关于catkin CMakeList.txt的介绍，**[官网原文链接](http://wiki.ros.org/catkin/CMakeLists.txt)**，由于直接阅读英文文档感觉自己理解不透彻、收获不多，所以决定一边翻译一边学习。其中零星的加入了一些译者个人使用过程中的体会以及在阅读《机器人操作系统（ROS）浅析》（Jason M.O’Kane著 肖军浩译）一书时学习到的内容，帮助自己更好地理解catkin编译生成的过程，留作今后复习完善。

## 概况

​	CMakeList.txt文件是CMake编译系统编译软件包过程的输入文件。任何CMake兼容包都包含一个或多个CMakeLists.txt文件，这些文件描述了如何编译代码以及将其安装到哪里。将CMakeLists.txt文件应用于一个catkin项目时，它就作为**一个标准的附带一些限制条件的vanilla CMakeLists.txt文件。**使用CMake编译程序时，`cmake`指令依据CMakeLists.txt 文件生成makefiles文件，`make`命令再依据makefiles文件编译链接生成可执行文件。

​	catkin是ROS官方的一个编译构建系统，是原本的ROS的编译构建系统rosbuild的发展。`catkin_make`是将`cmake`与`make`的编译方式做了一个封装的指令工具，规范了工作路径与生成文件路径。



 ## 总体结构和顺序

​	CMakeList.txt文件必须遵循如下的格式，不然就无法正确地编译（译者遇到一些编译ros软件包时提示“ros未定义的引用”的错误，原因就是CMakeList.txt文件中命令顺序不正确）。

- 必需的CMake版本：`cmake_minimum_required()`
- 软件包名：`project()`
- 查找编译依赖的其他CMake/Catkin包（声明依赖库）：`find_package()`
- 启动Python模块支持：`catkin_python_package()`
- 消息/服务/操作(Message/Service/Action)生成器：`add_message_files()`,`add_service_files()`,`add_action_files()`
- 调用消息/服务/操作生成：`generate_messages()`
- 指定包编译信息导出：`catkin_package()`
- 添加要编译的库和可执行文件：`add_library()`/`add_executable()`/`target_link_libraries()`
- 测试编译：`catkin_add_gtest()`
- 安装规则：`install()`

## CMake版本

​	每一个catkin CMakeList.txt文件必须以所需的CMake版本说明语句开始，Catkin需要2.8.3或者更高的版本

~~~cmake
cmake_minimum_required(VERSION 2.8.3)
~~~

## 软件包包名

​	软件包报名使用CMake的   `project()`函数指明，例如以robot_brain命名一个软件包：

~~~cmake
project(robot_brain)
~~~

​	CMake中，可以通过使用变量	  `${PROJECT_NAME}`在CMake脚本后面的任何位置引用项目名称。

## 查找编译依赖的CMake包

​	编译一个项目，需要使用CMake 的   `find_package`函数确定依赖的其他CMake包并找到它们，一般情况下至少会有一个catkin依赖：

~~~cmake
find_package(catkin REQUIRED)
~~~

​	除此之外，项目依赖的其他软件包，都会自动成为catkin的组件（components）（就CMake而言）。因此可以将这些依赖包指定为catkin的组件，而不必再使用`find_package`，这样将会变得简单，例如依赖包nodelet：

~~~cmake
find_package(catkin REQUIRED COMPONENTS nodelet)
~~~

​	**注意：只能`find_package`那些想要编译标志的组件，不能添加运行时（runtime）依赖。**

​	当然也可以写成下面的方式，但不方便:

~~~cmake
find_package(catkin REQUIRED)
find_package(nodelet REQUIRED)- ? 
~~~

### find_package()做了什么？

​	如果CMake通过  `find_package()`查找到一个软件包，它就会创建几个CMake环境变量，以提供有关已查找到的软件包的信息。这些环境变量可以在后面的CMake脚本中使用，它们表示软件包导出的头文件所在的位置、源文件所在的位置、软件包依赖的库以及这些库的查找路径，环境变量的名字遵循`<PACKAGENAME>_<PROPERTY>`，即包名-属性：

- `<NAME>_FOUND`：当库被查找到时置为true，否则为false
- `<NAME>_INCLUDE_DIRS`或`<NAME>_INCLUDES`：软件包导出的头文件路径
- `<NAME>_LIBRARIES`或`<NAME>_LIBS`：软件包导出的库的路径
- `<NAME>_DEFINITIONS`：？

### 为何将Catkin软件包指定为组件？

​	Catkin软件包严格意义上并不是catkin的组件，而且，CMake的功能组件功能被用于catkin的设计，以节省大量的打字时间。

​	对于catkin软件包，以catkin的组件的方式  `find_package`它们是有好处的，因为这个过程以catkin_prefix的形式创建了一组环境变量。例如，在程序中要使用nodelet软件包，推荐查找软件包的方式是：

~~~cmake
find_package(catkin REQUIRED COMPONENTS nodelet)
~~~

​	这就意味着nodelet导出的头文件路径、库等都会附加到  `catkin_variables`上，比如，`catkin_INCLUDE_DIRS`不仅包含catkin的头文件路径，也包含了nodelet软件包的头文件路径，这在后面会派上用场。

​	如果单独的`find_package nodelet`：

~~~cmake
find_package(nodelet)
~~~

​	这意味着nodelet的头文件路径、库及其他文件都不会包含在  `catkin_variables`中，对于`nodelet_INCLUDE_DIRS`,` nodelet_LIBRARIES`及其他变量也是如此。相同的变量也可以通过下面的方式创建：

~~~cmake
find_package(catkin REQUIRED COMPONENTS nodelet)
~~~

### Boost库

​	如果使用C++和Boost库，需要在Boost上调用  `find_package()`，并指定Boost中将要作为组件的那部分。例如，如果想要使用Boost的线程，可以用：

~~~cmake
find_package(Boost REQUIRED COMPONENTS thread)
~~~

## catkin_package()

​	`catkin_package()`是一个由catkin提供的CMake宏。需要指定特定的catkin信息到编译系统，而这些信息又会被用于生成pkg-config和CMake文件。

​	该函数必须在使用     `add_library()`或`add_executable()`声明任何targets之前调用。其5个可选参数：

- `INCLUDE_DIRS`：软件包导出的头文件路径（例如cflags）
- `LIBRARIES`：项目导出的库
- `CATKIN_DEPENDS`：当前项目依赖的其他catkin项目
- `DEPENDS`：当前项目依赖的非catkin CMake项目，详细解释参见[这里](answers.ros.org/question/58498/what-is-the-purpose-of-catkin_depends/)
- `CFG_EXTRAS`：其他的配置选项

		完整的宏文件参见[这里](#catkin-package)。

	例子：

~~~cmake
catkin_package( INCLUDE_DIRS include  
                LIBRARIES ${PROJECT_NAME}   
                CATKIN_DEPENDS roscpp nodelet   
                DEPENDS eigen opencv)
~~~

​	这里表明软件包文件夹中的include文件夹是导出头文件的位置，CMake环境变量   `${PROJECT_NAME}`将会鉴定之前传递给`project()`函数的所有内容，在这种情况下它作为“robot_brain”。“roscpp”+“nodelet”是编译/运行此程序包需要存在的软件包，“eigen”+“opencv”是编译/运行此程序包时需要存在的系统依赖项（ROS packages有时会需要操作系统提供一些外部函数库，这些函数库就是所谓的“系统依赖项”）。

## 明确编译的目标

​	编译目标可以采取多种形式，但通常它们代表两种可能性之一：	

- 可执行目标：可以运行的程序
- 库目标：在编译和/或运行时可以由可执行目标使用的库

### 目标命名

​	非常重要的一点是，不管编译/安装到哪个文件夹中，编译目标在catkin中的名称都必须是唯一的。这是CMake的一项要求，但目标唯一的名称又只是在CMake内部是必需的。可以使用`set_target_properties()`函数对目标重命名，例子：

~~~cmake
set_target_properties(rviz_image_view 
		      		  PROPERTIES OUTPUT_NAME image_view
                      PREFIX "")
~~~

​	这会在编译和安装输出中将目标     `rviz_image_view`的名称改为`image_view`。

### 自定义输出目录

​	可执行文件和库的默认输出目录通常设置为了合理的值，但在某些情况下必须进行自定义，例如，包含Python绑定的库必须放置在不同的文件夹中才能在Python中导入。

​	例子：

~~~cmake
set_target_properties(python_module_library  PROPERTIES LIBRARY_OUTPUT_DIRECTORY 								  {CATKIN_DEVEL_PREFIX}/{CATKIN_PACKAGE_PYTHON_DESTINATION})
~~~

### 头文件和库路径

​	在指定目标之前，需要指定可以为所述目标找到资源的位置，特别是头文件和库：

- 头文件目录：将要编译的代码（C/C++）所需的头文件路径	
- 库目录：可执行目标编译指向的库路径
- `include_directories(<dir1>, <dir2>, ..., <dirN>)`
- `link_directories(<dir1>, <dir2>, ..., <dirN>)`

#### include_directories()

​	`include_directories`的参数应该是由调用`find_package`生成的`* _INCLUDE_DIRS`变量以及需要包含的任何其他目录。如果使用`catkin`和`Boost`，`include_directories()`的调用为：

~~~cmake
include_directories(include {Boost_INCLUDE_DIRS} {catkin_INCLUDE_DIRS})
~~~

​	第一个参数“include”表示包中的include/目录也是路径的一部分。

#### link_directories()

​	CMake的   `link_directories()`函数可以添加其他的库目录，然而，并不推荐这么做。所有的catkin和CMake包在`find_package`时都会自动添加链接信息。只需链接到`target_link_libraries()`中的库。

​	例子：

~~~cmake
link_directories(~/my_libs)
~~~

​	详细信息参加[这里](http://www.cmake.org/pipermail/cmake/2011-May/044295.html)。

### 可执行目标

​	要指定必须编译的可执行目标，必须使用CMake函数   `add_executable()`。声明想要的可执行文件的文件名，以此生成此可执行文件所需的源文件列表，如果有多个源文件，用空格区分开。例如：

~~~cmake
add_executable(myProgram src/main.cpp src/some_file.cpp src/another_file.cpp)
~~~

​	该命令会编译名为   `myProgram`的可执行文件，它是由后面的三个源文件共同编译生成的。

### 库目标

​	CMake函数   `add_library()`指定用于编译的库文件，默认情况下，catkin编译共享库。	

~~~cmake
add_library({PROJECT_NAME} {${PROJECT_NAME}_SRCS})
~~~

## target_link_libraries

​	使用   `target_link_libraries()`函数指定可执行目标所要链接的库，即告诉CMake当链接此可执行文件时需要链接哪些库（这些库在上面的`find_package`中定义），通常在调用完`add_executable()`后被调用。如果出现[ros未定义的引用](#post-id-63674)错误，则添加`${catkin_LIBRARIES}`。

​	语法：

~~~cmake
target_link_libraries(<executableTargetName>, <lib1>, <lib2>, ... <libN>)
~~~

​	例子:

~~~cmake
add_executable(foo src/foo.cpp)
add_library(moo src/moo.cpp)
target_link_libraries(foo moo) 
~~~

​	上面的例子将     `foo`与`libmoo.so`链接起来

​	**注意，在大多数使用情况下，没有必要使用`link_directories()`，因为该信息通过`find_package()`已经自动提取到了。** 

## 消息、服务和操作目标

​	在被ROS软件包编译和使用之前，ROS中的消息（.msg）、服务（.srv）和操作（.action）文件需要特殊的预处理器编译步骤。这些宏的要点是生成编程语言特定的文件，以便可以在编程语言中使用消息、服务和操作。编译系统将使用所有可用的生成器（例如gencpp、genpy、genlisp）生成绑定。

​	提供了三个宏来分别处理消息，服务和操作：

- add_message_files
- add_service_files
- add_action_files

		这些宏后面必须调用一个调用生成的宏：

~~~cmake
generate_messages()
~~~

### 重要的前提和限制

1. 这些宏必须在调用`catkin_package()`之前被调用，以正确地完成生成工作。

~~~cmake
find_package(catkin REQUIRED COMPONENTS ...) 
add_message_files(...) 
add_service_files(...) 
add_action_files(...) 
generate_messages(...) 
catkin_package(...) ...
~~~

2. `catkin_package()`宏必须包含一个在`message_runtime`上的`CATKIN_DEPENDS`依赖。

~~~cmake
catkin_package( ... 
                CATKIN_DEPENDS message_runtime ... 
                ...)
~~~

3. 必须对软件包`message_generation`使用`find_package()`，可单独或者作为catkin的组件使用：

~~~cmake
find_package(catkin REQUIRED COMPONENTS message_generation)
~~~

4. `package.xml`文件必须包含一个在`message_generation`上的编译依赖和一个在`message_runtime`上的运行时依赖，如果从其他包中传递依赖关系，则这不是必需的。
5. 如果有一个目标（甚至是过渡性的）依赖于需要建立消息/服务/动作的其他目标，需要在目标`catkin_EXPORTED_TARGETS`上添加显式依赖项，以使它们按照正确的顺序编译。这种情况几乎总是适用，除非你的软件包真的不使用ROS的任何部分。不幸的是，这种依赖不能自动传播。（some_target是由`add_executable()`设置的目标的名字）

~~~cmake
add_dependencies(some_target ${catkin_EXPORTED_TARGETS})
~~~

6. 如果有编译消息和/或服务的软件包以及使用这些软件的可执行文件，则需要在自动生成的消息目标上创建明确的依赖关系，以便它们按正确的顺序编译。（some_target是由`add_executable()`设置的目标的名字）

~~~cmake
add_dependencies(some_target ${${PROJECT_NAME}_EXPORTED_TARGETS})
~~~

7. 如果软件包满足上述两个条件，则需要添加两个依赖项，即：

~~~cmake
add_dependencies(some_target {${PROJECT_NAME}_EXPORTED_TARGETS}   		      	 {catkin_EXPORTED_TARGETS})
~~~

### 例子

​	如果在msg目录下有两个消息文件       `MyMessage1.msg`和`MyMessage2.msg`，并且这些消息依赖于`std_msgs`和`sensor_msgs`，另外在srv目录下有一个服务文件`MyService.srv`，就可以使用这些消息、服务定义可执行`message_program`，和可执行的程序`does_not_use_local_messages_program`，这个过程使用了ROS的某些部分，但不包含此包中定义的消息/服务。需要在CMakeList.txt文件中加上一下内容：

~~~cmake
# Get the information about this package's buildtime dependencies  find_package(catkin REQUIRED    COMPONENTS message_generation std_msgs sensor_msgs)  
# Declare the message files to be built  
add_message_files(FILES    MyMessage1.msg    MyMessage2.msg  )  
# Declare the service files to be built  add_service_files(FILES    MyService.srv  )  
# Actually generate the language-specific message and service files  generate_messages(DEPENDENCIES std_msgs sensor_msgs)  
# Declare that this catkin package's runtime dependencies  catkin_package(   CATKIN_DEPENDS message_runtime std_msgs sensor_msgs  )  
# define executable using MyMessage1 etc.  add_executable(message_program src/main.cpp)  add_dependencies(message_program {${PROJECT_NAME}_EXPORTED_TARGETS} {catkin_EXPORTED_TARGETS})  
# define executable not using any messages/services provided by this package  
add_executable(does_not_use_local_messages_program src/main.cpp)  add_dependencies(does_not_use_local_messages_program ${catkin_EXPORTED_TARGETS})
~~~

​	另外如果需要编译actionlib操作，并且在action目录下有一个名为`MyAction.action`的操作规范文件，就必须要添加`actionlib_msgs`到组件列表中，该组件列表就是`find_package`中catkin的组件，并在调用`generate_messages()`之前调用：

~~~cmake
add_action_files(FILES MyAction.action)
~~~

​	此外，该包必须对   `actionlib_msgs`具有编译依赖关系。

## 启动Python模块支持

​	如果ROS软件包提供了一些Python模块，就要创建一个`setup.py`文件并调用：

~~~cmake
catkin_python_setup()
~~~

​	该调用要在`generate_message()`和`catkin_package()`的调用之前。

## 单元测试

​	特定的catkin宏   `catkin_add_gtest()`用于处理基于gtest的单元测试：

~~~cmake
catkin_add_gtest(myUnitTest test/utest.cpp)
~~~

## 可选步骤：明确安装目标

​	编译完成后，目标被放入catkin工作空间下的devel目录。一般希望将目标安装到系统上，以使其他用户使用，或者安装到本地目录来测试系统级别的安装。也就是说，如果希望能够对代码进行`make install`，就需要明确目标结束的位置。

​	上述过程可以使用CMake的   `install()`函数实现，该函数的参数有：

- `TARGETS`：要安装的目标
- `ARCHIVE DESTINATION`：静态库和动态链接库DLL(Windows).lib存根
- `LIBRARY DESTINATION`：非DLL共享库和模块
- `RUNTIME DESTINATION`：可执行目标和DLL(Windows)模式共享库

		例子：

~~~cmake
install(TARGETS ${PROJECT_NAME}  
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}  
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}  
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
~~~

​	除了这些标准的目标，还要安装一些文件到特定的目录下，即一个包含Python绑定的库必须要安装到另外的不同的目录下，这对Python是重要的：

~~~cmake
install(TARGETS python_module_library  
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}  
        LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
~~~

### 安装Python可执行脚本

​	Python代码的安装规则有些不同，它不需要使用     `add_library()`和`add_executable()`函数来告知CMake哪个文件是目标文件、目标文件是什么类型的。而是使用如下的CMakeList.txt文件：

~~~cmake
catkin_install_python(PROGRAMS scripts/myscript  
					  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
~~~

​	如果只是安装了Python的脚本，不提供任何模块的话，就不用创建上文提到的   `setup.py`文件，也不用调用`catkin_python_setup()`。

### 安装头文件

​	头文件必须安装到include目录下，这通常通过安装整个文件夹的文件来完成（可以根据文件名模式进行过滤，并排除SVN子文件夹）。可以通过一下安装规则实现：

~~~cmake
install(DIRECTORY include/${PROJECT_NAME}/  
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}  
        PATTERN ".svn" EXCLUDE)
~~~

​	或者如果include目录下的子文件夹无法和软件包名匹配时：

~~~cmake
install(DIRECTORY include/  
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}  
        PATTERN ".svn" EXCLUDE)
~~~

### 安装roslaunch文件或其他源

​	其他像launchfiles的资源可以安装到   `${CATKIN_PACKAGE_SHARE_DESTINATION}`：

~~~cmake
install(DIRECTORY launch/  
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch  
        PATTERN ".svn" EXCLUDE)
~~~
## CMakeLists.txt文件书写模板

~~~cmake
cmake_minimum_required(VERSION 2.8.3)
project(my_p)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_p
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/my_p.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/my_p_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_p.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
~~~



