---
title: Docker学习之基础知识
date: 2018-05-13 21:31:07
tags:
   - Docker
categories: Docker
copyright: true
---

-----

这篇文章是有关Docker基础知识的内容。

<!--more--->

官方文档：https://docs.docker.com/install/linux/docker-ce/ubuntu/

学习到这啦：Containers share your image：https://docs.docker.com/get-started/part2/#share-your-image

Dockerfile：https://docs.docker.com/engine/reference/builder/#usage

Docker在乌班图系统上支持overlay2和aufs两种存储驱动。并且默认使用overlay2，如果需要使用aufs，需要手动配置。

Docker是一个为开发者和系统管理员提供的平台，允许使用容器来开发、部署、运行应用。

Docker的文件系统分为两层：bootfs和rootfs。

{% asset_img bootfs.jpg %}

Docker采用AUFS分层文件系统时，文件系统的改动都是发生在最上面的容器层。在容器的生命周期内，它是持续的，包括容器在被停止后。但容器被删除后，该数据层也随之被删除了。因此Docker使用volume（卷）的形式来向容器提供持久化存储。Docker使用UnionFS搭建的分层镜像：

{% asset_img docker-filesystems-multilayer.png %}

## 概念

- 容器（container）：是一个镜像的实例，通过运行一个镜像启动。它是一个镜像的运行时实例，这时，执行的镜像位于内存中。可以使用docker ps命令查看运行中的容器清单。运行着的容器有一个可写层（writable或称为容器层container layer），位于底下的若干只读层之上，运行时的所有变化，包括对数据和文件的写和更新，都会保存在这个层中。因此，从同一个镜像运行的多个容器包含了不同的容器层。
- 镜像（image）：是一个可执行包，包括执行一个应用程序的所有代码、运行时、库、环境变量以及配置文件。镜像是轻便的，它由Dockerfile定义。镜像是文件系统数据的复制，只允许读，它包括一个或多个只读层（read-only layers），一旦被创建就无法修改。
- 仓库（repository）：镜像的集合，其中的代码已经编译完成。
- registry：仓库的集合。默认使用Docker的公共registry，也可以自己设置私人registry。
- Swarm：一些运行Docker并且加入到cluster中的机器的集合。

## Docker命令

~~~shell
docker --version 			#查看Docker版本
docker info 				#查看Docker安装有关的所有细节信息
docker version				#查看Docker安装有关的所有细节信息
docker image ls				#列出镜像清单
docker container ls 	 	#列出容器清单（列出运行中的容器）
docker container ls --all 	#列出容器清单（列出所有容器）
docker container ls --aq 	#列出容器清单（列出所有容器，简单模式，只有容器ID）
docker run hello-world		#执行Docker镜像，镜像名字为hello-world

~~~

## 分层结构

- Stack：一组有关联的服务的组合，可以编排在一起，一起管理。
- Services：一个应用的不同部分。伸缩一个服务就是改变这一个服务的运行的容器的数量。
- Container

## Flocker：容器的分布式存储平台

原生的 Docker volume 不具备可移植性。于是，出现了Docker 的分布式卷解决方案 [Flocker](https://github.com/ClusterHQ/flocker)。Flocker的结构：

{% asset_img flocker.jpg %}

参考文章：http://www.cnblogs.com/sammyliu/p/5932996.html