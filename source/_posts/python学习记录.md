---
title: python学习记录
date: 2019-06-03 23:02:49
tags:
  - Python
categories: 
  - 语言
  - Python
copyright: true
---
---

-

<!--more--->

查看python的路径：

```
whereis python
```

ubuntu下切换python版本：

https://blog.csdn.net/beijiu5854/article/details/77897767

anaconda和ROS共存：

https://www.jianshu.com/p/4e437e25480b

tensorflow环境搭建过程：

- 安装virtualenv：`sudo apt-get install python-virtualenv`
- `mkdir envs`
- `cd evns`
- 创建虚拟环境：`virtualenv -p /usr/bin/python3 tensorflow`
- 激活虚拟环境：`source ~/envs/tensorflow/bin/activate`
- 安装tensorflow：`pip install tensorflow`