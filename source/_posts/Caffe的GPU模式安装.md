---
title: Caffe的GPU模式安装
date: 2018-09-08 12:47:11
tags:
  - Caffe
categories: 
  - 深度学习
  - Caffe
copyright: true
---

---

这篇文章是有关Caffe GPU模式安装的内容。

<!--more--->

暑假终于入手了一台属于自己的可以称之为高配置的笔记本，实验室的台式机显卡太渣，不想耗费时间和精力再配置台式机，自己的笔记本随时随地方便使用，鼓捣好自己的笔记本环境配置才是王道。

## 笔记本配置

- CPU-Intel七代i7
- 内存-16G 显存6G
- 双显卡-GeForce GTX 1060 + Intel
- 双系统Win10+Ubuntu16.04 

## 相关链接

1. [查看自己的GPU是否支持CUDA](<https://developer.nvidia.com/cuda-gpus>)
2. [下载匹配自己显卡的驱动](https://www.nvidia.cn/Download/index.aspx?lang=cn)(apt方法安装则不需要)
3. [CUDA下载](https://developer.nvidia.com/cuda-toolkit-archive) 选择合适的版本，选择下载runfile文件
4. [下载cuDNN library](https://developer.nvidia.com/rdp/cudnn-archive) 需要注册帐号
5. [NVIDIA CUDA Installation Guide for Linux](http://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#axzz4dMB1vD9s)
6. [Caffe1官网](http://caffe.berkeleyvision.org/)
7. [Caffe源码](https://github.com/BVLC/caffe)

## 具体过程

- 安装Nvidia显卡驱动
- 安装CUDA9
- 安装cuDNN
- 安装Caffe

### 安装Nvidia显卡驱动

1. 从[相关链接2](https://www.nvidia.cn/Download/index.aspx?lang=cn)查询到自己系统对应的版本

2. 安装驱动

   ~~~shell
   sudo add-apt-repository ppa:graphics-drivers/ppa  
   sudo apt-get update  
   sudo apt-get install nvidia-390 #此处要根据上面查询到的版本适当更改
   sudo apt-get install mesa-common-dev  
   sudo apt-get install freeglut3-dev
   ~~~

   之后重启系统让GTX1060显卡驱动生效。

3. 测试。终端输入`nvidia-smi`，会显示显卡相关的信息，说明安装驱动成功。

   {% asset_img nvidia安装成功.png %}

4. 可以打开Nvidia x server setting切换双显卡。

> ### 可能遇到的问题
>
> 执行`sudo apt-get install nvidia-390`命令时可能会产生依赖包冲突的问题。
>
> 解决方法：
>
> 使用aptitude安装，首先安装apitude`sudo apt-get install aptitude`，使用apitude进行安装的命令`sudo aptitude install xxxx`。根据提示选Y/N/Q，通常选N直到出现对版本做降级处理，点Y即可解决。

### 安装CUDA9.0

1. 从[相关链接3](https://developer.nvidia.com/cuda-toolkit-archive)下载CUDA9.0，选择funfile（下载好的文件名：cuda_9.0.176_384.81_linux.run）

   {% asset_img CUDA.png %}

   > **注意：**CUDA要和显卡驱动对应，如下图。[参考链接](https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html)。
   >
   > {% asset_img CUDA和驱动版本对应.png %}

2. 执行命令`sudo ./cuda_9.0.176_384.81_linux.run`启动安装程序，一直按空格到最后，输入accept接受条款。

   - 输入n不安装nvidia图像驱动
   - 输入y安装cuda 9.0工具
   - 回车确认cuda默认安装路径：/usr/local/cuda-9.0
   - 输入y或者n安装或者不安装指向/usr/local/cuda的符号链接
   - 输入y安装CUDA 9.0 Samples，以便后面测试
   - 回车确认CUDA 9.0 Samples默认安装路径：/home/eric（eric是我的用户名），该安装路径测试完可以删除
   - 安装完显示如下图 

   {% asset_img CUDA9安装.png %}

   > 如果需要卸载CUDA：
   >
   > To uninstall the CUDA Toolkit, run the uninstall script in /usr/local/cuda-9.0/bin

3. 添加环境变量

   执行命令`sudo gedit /etc/profile`编辑文件，在最后添加：

   ~~~
   export PATH=/usr/local/cuda-9.0/bin:$PATH
   export LD_LIBRARY_PATH=/usr/local/cuda-9.0/lib64:/usr/local/cuda-9.0/extras/CUPTI/lib64:$LD_LIBRARY_PATH
   ~~~

   重启系统！

4. 测试CUDA Toolkit安装是否正确：`nvcc --version`，输出以下信息说明安装正确：

   {% asset_img CUDA测试.png %}

5. 编译CUDA Samples，默认路径为`~/NVIDIA_CUDA-9.0_Samples`

   ```
   make
   ```

   生成可执行文件在`~/NVIDIA_CUDA-9.0_Samples/bin/x84_64/linux/release`

   ```
   ./deviceQuery
   ```

   会有如下输出：

   {% asset_img samples.png %}

   > 如果在第一步下载的CUDA和显卡驱动不对应的话，会提示：
   >
   > ~~~
   >  CUDA driver version is insufficient for CUDA runtime version
   > ~~~

### 安装cuDNN

cuDNN的全称为NVIDIA CUDA® Deep Neural Network library，是NVIDIA专门针对深度神经网络（Deep Neural Networks）中的基础操作而设计基于GPU的加速库。cuDNN为深度神经网络中的标准流程提供了高度优化的实现方式，例如convolution、pooling、normalization以及activation layers的前向以及后向过程。

1. 从[相关链接4](https://developer.nvidia.com/rdp/cudnn-archive)下载合适版本的cuDNN（下载好的文件名：cuda_9.0.176_384.81_linux.run）

   {% asset_img cuDNN.png %}

2. 解压cuda_9.0.176_384.81_linux.run

   ~~~shell
   tar -xvzf cuda_9.0.176_384.81_linux.run
   sudo cp cuda/include/cudnn.h /usr/local/cuda/include
   sudo cp cuda/lib64/libcudnn* /usr/local/cuda/lib64
   sudo chmod a+r /usr/local/cuda/include/cudnn.h /usr/local/cuda/lib64/libcudnn*
   ~~~

3. 更新软链接（应该不需要这一步）

   ~~~shell
   cd /usr/local/cuda/lib64/
   sudo rm -rf libcudnn.so libcudnn.so.5
   sudo ln –s libcudnn.so.5.1.10 libcudnn.so.5
   sudo ln –s libcudnn.so.5 libcudnn.so
   ~~~


### 安装Caffe

1. 安装依赖包

   ~~~shell
   sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
   sudo apt-get install --no-install-recommends libboost-all-dev
   sudo apt-get install libatlas-base-dev
   sudo apt-get install build-essential
   sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev
   ~~~

2. 安装python的pip和easy_install，方便安装软件包

   ~~~shell
   cd
   wget --no-check-certificate https://bootstrap.pypa.io/ez_setup.py
   sudo python ez_setup.py --insecure
   wget https://bootstrap.pypa.io/get-pip.py
   sudo python get-pip.py
   ~~~

3. 安装科学计算和python所需的部分库

   ~~~shell
   sudo apt-get install libblas-dev liblapack-dev libatlas-base-dev gfortran python-numpy
   ~~~

4. 安装python依赖

   ~~~shell
   sudo apt-get install python-pip #安装pip
   ~~~

5. 编译Caffe

   ##### 终端输入

   ~~~shell
   cd /home/eric/caffe
   cp Makefile.config.example Makefile.config
   gedit Makefile.config
   ~~~

   - 将`USE_CUDNN := 1`取消注释
   - `INCLUDE_DIRS := $(PYTHON_INCLUDE) /usr/local/include`后面打上一个空格 然后添加`/usr/include/hdf5/serial`如果没有这一句可能会报一个找不到hdf5.h的错误

   ##### 终端输入 

   ~~~shell
   make
   ~~~

   > #### make过程中出现找不到lhdf5_hl和lhdf5的错误
   >
   > 解决方案：在计算机中搜索`libhdf5_serial.so.10.1.0`，找到后右键点击打开项目位置。该目录下空白处右键点击在终端打开，打开新终端输入   `sudo ln libhdf5_serial.so.10.1.0 libhdf5.so`   `sudo ln libhdf5_serial_hl.so.10.0.2 libhdf5_hl.so` 。最后在终端输入`sudo ldconfig`使链接生效，原终端中输入`make clean`清除第一次编译结果，再重新编译。
   >
   > #### 出现`nvcc fatal   : Unsupported gpu architecture 'compute_20'`的错误
   >
   > 将Makefile.config文件中`CUDA_ARCH :=`包含`compute_20`的两项删除即可。

   ##### 终端输入：

   ~~~shell
   make test -j4
   make runtest -j4
   make pycaffe -j4
   make distribute
   ~~~

   生成发布安装包

   ##### 测试python，终端输入:

   ~~~shell
   cd /home/erci/install/caffe/python
   python
   import caffe
   ~~~

   如果不报错就说明编译成功。

   > #### 提示
   >
   > 如果执行`import caffe`，出现错误`ImportError: No module named skimage.io`，可以进行如下操作：
   >
   > - `sudo apt-get install python-skimage`
   > - `sudo apt-get install python-numpy python-scipy python-matplotlib python-sklearn python-skimage python-h5py python-protobuf python-leveldb python-networkx python-nose python-pandas python-glags ipython`
   > - `sudo apt-get update`
   > - caffe目录下：`make pycaffe`

## 参考资料

1. [[专业亲测]Ubuntu16.04安装Nvidia显卡驱动（cuda）--解决你的所有困惑](https://blog.csdn.net/ghw15221836342/article/details/79571559)
2. [ubuntu16.04+gtx1060+cuda8.0+caffe安装、测试经历](https://blog.csdn.net/wopawn/article/details/52302164)
3. [Ubuntu16.04+GTX1050+CUDA8.0配置深度学习环境](https://blog.csdn.net/sikao_luwei/article/details/69375126)
4. [ubuntu16.04下软件依赖冲突的解决方案](https://blog.csdn.net/qq_36511757/article/details/79795013)
5. [cuDNN安装官网教程](https://docs.nvidia.com/deeplearning/sdk/cudnn-install/index.html#install-linux)