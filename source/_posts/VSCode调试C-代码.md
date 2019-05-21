---
title: VSCode调试C++代码
date: 2019-04-29 18:01:15
tags:
  - ubuntu
  - VSCode
  - C++
  - Debug
categories:
  - 工具
  - VSCode
copyright: true
---
---

简单记录下使用VSCode调试代码收集的一些内容。
<!--more--->

1. 对于一般的C++程序，可以参考[这里](https://medium.com/@LicHacker/debugging-c-with-vscode-and-gdb-a266eec287e3)或[这里](https://blog.csdn.net/weixin_43374723/article/details/84064644#5_129)配置相关文件，并调试代码。
   - 其中比较重要的一步是在`launch.json`文件中添加可执行文件的路径；
   - 如果不想用外部控制台进行调试，只在vscode内部显示相关信息，设置参数`“externalConsole”: false`即可。
   - [GDB Quick Guide](https://www.tutorialspoint.com/gnu_debugger/gdb_quick_guide.htm)
2. 对于采用CMake编译的情况，除了步骤1的配置外，还需注意：
   - CMakeList.txt文件`set`命令中需要添加`-g`，表示允许调试，否则即使设置了断点，也不会在断点处暂停。例如：`set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -march=native -g" )`。参考自[链接1](https://stackoverflow.com/questions/50306354/fail-to-hit-breakpoint-of-c-program-build-with-cmake-on-ubuntu)、[链接2](https://github.com/Microsoft/vscode-cpptools/issues/416)。
3. 对于ROS程序，除了上述步骤1、2的配置外，还需注意：
   - 项目`launch.json`文件中添加的可执行文件路径为：`/home/username/ros_work_space/devel/lib/ros_package_name/file_name`。
   - 关于ROS下如何调试程序，可参见[这里](http://www.dataguru.cn/article-10359-1.html)。
4. ROS程序，如果使用`roslaunch`命令启动节点
   - 需要在`launch`文件中节点定义一行添加：`launch-prefix="xterm -e gdb --args" `，例如：`<node name="x" pkg="xx" type="xxx" output="screen" launch-prefix="xterm -e gdb --args" >`。
   - [Roslaunch Nodes in valgrind or GDB](http://library.isr.ist.utl.pt/docs/roswiki/roslaunch(2f)Tutorials(2f)Roslaunch(20)Nodes(20)in(20)Valgrind(20)or(20)GDB.html)