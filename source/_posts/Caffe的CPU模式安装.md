---
title: Caffe的CPU模式安装
date: 2018-04-21 15:20:24
tags:
  - Caffe
categories: 
  - 深度学习
  - Caffe
---

-----

这篇文章是有关Caffe CPU模式安装的内容。

<!--more--->

在跑师兄的程序时，有一个场景检测模块用到Caffe深度学习框架库，可惜自己的台式机显卡太菜，装不了CUDA。只能尝试使用Caffe的CPU模式。Caffe在计算时有两种模式可以选择，CPU或GPU，使用GPU处理图像速度会更快。

## 检查是否有NVIDIA显卡

~~~shell
lspci | grep -i nvidia
~~~

如果没有显示内容，说明电脑没有nvidia显卡。如果输出相关显卡及版本信息，说明可以使用GPU模式。

## 安装依赖包

~~~shell
sudo apt-get install libprotobuf-dev 
sudo apt-get install libleveldb-dev
sudo apt-get install libsnappy-dev 
sudo apt-get install libopencv-dev
sudo apt-get install libhdf5-serial-dev
sudo apt-get install protobuf-compiler
sudo apt-get install libgflags-dev
sudo apt-get install libgoogle-glog-dev
sudo apt-get install liblmdb-dev
sudo apt-get install libatlas-base-dev
~~~

## 下载Caffe

安装git

~~~shell
sudo apt-get install git
~~~

克隆Caffe

~~~shell
git clone git://github.com/BVLC/caffe.git
~~~

## 编译Caffe

### 进入Caffe目录

~~~shell
cd caffe/
~~~

### 生成Makefile.config文件

将caffe目录下自带的Makefile.config.example文件复制一份并更名为Makefile.config，命令如下：

~~~shell
cp Makefile.config.example Makefile.config
~~~

### 修改Makefile.config文件

- 取消`CPU_ONLY := 1`行的注释，设置为CPU模式
- 配置引用文件路径
  - `INCLUDE_DIRS`新增内容：`/usr/include/hdf5/serial`
  - `LIBRARY_DIRS`新增内容：`/usr/lib/x86_64-linux-gnu/hdf5/serial`

### 编译

需要在caffe目录下新建`build`目录，命令如下：

~~~shell
mkdir build
cd build
camke ..
sudo make all
sudo make test
sudo make runtest
~~~

编译成功的话，就会显示若干个用例执行成功。

可以执行`sudo make clean`撤销执行。

本人开始并没有新建build目录，直接开始`make`的。但是出现了类似如下的错误：

~~~shell
Error: 'make all' 'make test'
.build_release/lib/libcaffe.so: undefined reference to cv::imread(cv::String const&, int)' 
.build_release/lib/libcaffe.so: undefined reference tocv::imencode(cv::String const&, cv::_InputArray const&, std::vector >&, std::vector > const&)'
~~~

网上有很多解决方式，本人试过都没有效果，最后按照上述过程编译就不会有这个错误。



参考文章：

https://blog.csdn.net/u010193446/article/details/53259294

https://www.cnblogs.com/empty16/p/4828476.html?utm_source=tuicool&utm_medium=referral