---
title: coding.net git
date: 2018-08-02 16:50:09
tags:
---

-----

<!--more--->

# Git

{% asset_img git.png  %}

- Directory：使用Git管理的一个目录，也就是一个仓库，包含我们的工作空间和Git的管理空间。
- WorkSpace：需要通过Git进行版本控制的目录和文件，这些目录和文件组成了工作空间。
- .git：存放Git管理信息的目录，初始化仓库的时候自动创建。
- Index/Stage：暂存区，或者叫待提交更新区，在提交进入repo之前，我们可以把所有的更新放在暂存区。
- Local Repo：本地仓库，一个存放在本地的版本库；HEAD会只是当前的开发分支（branch）。
- Stash：是一个工作状态保存栈，用于保存/恢复WorkSpace中的临时状态。

{% asset_img git2.png  %}

## 命令

~~~shell
git init 					  	#创建本地仓库，自动产生'.git'隐藏文件
git status 					  	#查看workspace状态
git add filename or git add . 	#将文件或更新放到暂存区
git commit -m "message"			#提交更新，从暂存区到本地仓库，-m后是描述信息
git commit -a -m "message"		#跳过git add步骤提交更新，修改某个文件后提交可以执行该命令
git diff 					  	#显示workspace与暂存区的差异
git diff HEAD~n					#显示workspace与本地仓库的差异
git remote						#显示已经配置的远程仓库服务器
git remote -v 					#显示远程仓库服务器简写及对应的URL
git remote add shortname url	#添加远程仓库
git push shortname branchname	#推送数据到远程仓库
git log 						#查看提交历史
git log -p -2					#-p显示每次提交的内容差异，-2显示最近两次提交
~~~



ubuntu git远程仓库管理：https://blog.csdn.net/maclechan/article/details/44964439

详细的git原理及使用 ：https://git-scm.com/book/zh/v2

简易指南：http://www.bootcss.com/p/git-guide/