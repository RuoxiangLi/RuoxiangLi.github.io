---
title: Zotero基于坚果云实现多机同步
date: 2018-08-10 22:59:43
tags:
  - Zotero
categories: 工具
---

-----

这篇文章是有关Zotero工具基于坚果云和webdav实现多机同步的内容。

<!--more-->

首先参考[该文章](http://www.sohu.com/a/145196470_241268)在Zotero中设置webdav连接到坚果云，其中输入的坚果云服务器地址为`https://dav.jianguoyun.com/dav/work`，需要提前在坚果云创建`work`文件夹。

> 提示：输入的用户名为坚果云账号邮箱，密码为第三方应用管理中获取到的应用密码。

## Ubuntu设置方法

1. 我的zotero源文件开始是在ubuntu系统下，在上面的操作后，坚果云会提示在本地创建`work`目录，我设置在`/home/usrname/work/`；
2. zotero安装在`/home/usrname/Zotero`目录下，将该目录下的storage目录剪切到`/home/usrname/work/zotero`目录下；
3. 命令行执行：`ln -s /home/usrname/Zotero /home/usrname/work/zotero`，会在`Zotero`下面会出现软链接目录`storage`；
4. 等待坚果云同步完成，再回到zotero里面点击同步按钮。

> 提示：如果work目录已经创建好，里面同步了数据，这时候就可以直接将zotero下的storage删除，执行命令：
>
> `ln -s /home/eric/work/zotero/storage /home/eric/Zotero`创建软链接到zotero下即可。

## win设置方法

1. 由于前面在ubuntu系统已经设置过，进入windows后，坚果云会提示同步`work`目录（联网状态），这时选择同步到自己想要放置的磁盘，如`D:\work`。zotero默认安装在C盘，`storage`目录地址是`C:\User\user\Zotero\storage`；
2. 剪切`C:\User\user\Zotero\storage`目录到`D:\work\Zotero`目录下（这时zotero下面没有storage目录了）；
3. 运行cmd输入：`mklink /j C:\User\user\Zotero\storage D:\work\Zotero`，现在zotero下面出现了软链接目录`storage`。

参考：http://www.360doc.com/content/18/0506/18/46033958_751645308.shtml