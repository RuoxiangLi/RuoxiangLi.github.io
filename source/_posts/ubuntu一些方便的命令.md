---
title: ubuntu一些方便的命令
date: 2018-04-21 11:12:32
tags:
  - ubuntu
categories: 
  - 系统
  - ubuntu
---

-----

这篇文章是有关ubuntu下一些常用命令行指令的内容。

<!--more--->

### 查看opencv的版本

~~~shell
pkg-config --modversion opencv
~~~

### 查看显卡信息

~~~shell
lspci |grep VGA
~~~

### 显示设备信息

~~~shell
lspci
~~~

### 查看ubuntu系统版本

~~~shell
lsb_release -a
~~~

### 查看内核版本

~~~shell
uname -a
~~~

或

~~~shell
cat /proc/version
~~~

### 查看nvidia显卡信息

~~~
nvidia-smi
nvidia-settings
~~~

