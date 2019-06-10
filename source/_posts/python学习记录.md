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

在`~/.bashrc`文件中添加命令：

`alias tensorflow="source ~/envs/tensorflow/bin/activate"`

这样就可以在shell中使用`tensorflow`命令激活虚拟环境。

在tensorflow虚拟环境中搭建Jupyter Notebook环境

- 激活virtualenv：`tensorflow`
- 执行：`sudo pip3 install jupyter ipython`
- 执行：`sudo pip3 install ipykernel`
- 将 Virtualenv 加入IPykernel中：`sudo python3 -m ipykernel install --user --name tf1 --display-name 'Python(tf1)'`，其中`--name your-name1`是给Jupter启动Kernel使用的名字，`--display-name 'your-name2'`是Jupyter notebook 菜单显示的名字。（不使用`sudo`可能会出现错误`/home/eric/envs/tensorflow/bin/python3: No module named ipykernel`）
- 启动：`jupyter notebook`，shell中会有类似提示`本程序运行在: http://localhost:8888/?token=xxxxx`，点击在浏览器打开，并选择所需要的内核`Python(tf1)`，即`'your-name2'`。