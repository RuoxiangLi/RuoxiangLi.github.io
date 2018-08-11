---
title: CMake学习之查找链接库--find_package使用方法
date: 2018-08-03 21:38:19
tags:
   - CMake
categories: CMake
---

------

这篇文章是有关CMake中使用find_package指令的内容。

<!--more--->

**如果编译软件使用了外部库，事先并不知道它的头文件和链接库的位置。得在编译命令中加上包含它们的查找路径。CMake使用`find_package`命令来解决这个问题。本文讨论了如何在CMake项目中使用外部库，即`find_package()`的工作原理。**

## FIND_PACKAGE

`FIND_PACKAGE( <name> [version][EXACT] [QUIET][NO_MODULE] [ [ REQUIRED | COMPONENTS ][ componets... ] ] )`

用来调用预定义在 CMAKE_MODULE_PATH 下的 `Find<name>.cmake `模块。也可以自己定义` Find<name>`模块，将其放入工程的某个目录中，通过` SET(CMAKE_MODULE_PATH dir)`设置查找路径，供工程`FIND_PACKAGE`使用。

这条命令执行后，CMake 会到变量 CMAKE_MODULE_PATH 指示的目录中查找文件 `Find<name>.cmake `并执行。

- version参数：需要一个版本号，它是正在查找的包应该兼容的版本号。
- EXACT选项：要求版本号必须精确匹配。如果在find-module内部对该命令的递归调用没有给定[version]参数，那么[version]和EXACT选项会自动地从外部调用前向继承。对版本的支持目前只存在于包和包之间（详见下文）。

- QUIET 参数：会禁掉包没有被发现时的警告信息。对应于`Find<name>.cmake`模块中的 NAME_FIND_QUIETLY。
- REQUIRED 参数：其含义是指是否是工程必须的，表示如果报没有找到的话，cmake的过程会终止，并输出警告信息。对应于`Find<name>.cmake`模块中的 NAME_FIND_REQUIRED 变量。
- COMPONENTS参数：在REQUIRED选项之后，或者如果没有指定REQUIRED选项但是指定了COMPONENTS选项，在它们的后面可以列出一些与包相关（依赖）的部件清单（components list）

示例：

FIND_PACKAGE( libdb_cxx REQUIRED)

这条命令执行后，CMake 会到变量 CMAKE_MODULE_PATH 指示的目录中查找文件 Findlibdb_cxx.cmake 并执行。

## 包查找是如何工作的

1. `find_package()` 命令会在模块路径中寻找`Find<name>.cmake` ，这是查找库的一个典型方式。首先CMake查看`${CMAKE_MODULE_PATH}`中的所有目录，然后再查看它自己的模块目录`<CMAKE_ROOT>/share/cmake-x.y/Modules/`（`$CMAKE_ROOT`的具体值可以通过CMake中`message`命令输出）。**这称为模块模式。**
2. 如果没找到这样的文件，在`~/.cmake/packages/`或`/usr/local/share/`中的各个包目录中查找，寻找`<库名字的大写>Config.cmake`或者`<库名字的小写>-config.cmake`(比如库Opencv，它会查找`/usr/local/share/OpenCV`中的`OpenCVConfig.cmake`或`opencv-config.cmake`)。**这称为配置模式。**配置模式的文件的编写见 [这里的文档](http://vtk.org/Wiki/CMake/Tutorials/How_to_create_a_ProjectConfig.cmake_file) 。可能还会用到 [importing and exporting targets](http://vtk.org/Wiki/CMake/Tutorials/Exporting_and_Importing_Targets) 这篇文档。


不管使用哪一种模式，只要找到包，就会定义下面这些变量：

~~~
<NAME>_FOUND
<NAME>_INCLUDE_DIRS or <NAME>_INCLUDES
<NAME>_LIBRARIES or <NAME>_LIBRARIES or <NAME>_LIBS
<NAME>_DEFINITIONS
~~~

这些都在 `Find<name>.cmake`文件中。

找到NAME包后，变量NAME_INCLUDE_DIRS中将包括指定NAME库头文件的查找路径。
变量NAME_LIBRARY_DIRS中将包含指定NAME库的.a或.so文件的所在目录的路径。

参考链接：

http://blog.csdn.net/bytxl/article/details/50637277

https://blog.csdn.net/u011092188/article/details/61425924