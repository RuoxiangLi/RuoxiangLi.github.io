---
title: cv_brige
date: 2018-08-11 21:32:14
tags:
---

cv_bridge opencv image<-->ros message image

cv_bridge:toCvShare()
http://docs.ros.org/hydro/api/cv_bridge/html/c++/namespacecv__bridge.html#aafa38a1d9be98d9efaefe45fd873133c


https://www.cnblogs.com/li-yao7758258/p/6637079.html
图像编码可以是以下任何一个opencv图像编码：

    8UC[1-4]
    8SC[1-4]
    16UC[1-4]
    16SC[1-4]
    32SC[1-4]
    32FC[1-4]
    64FC[1-4]
介绍集中cvbridge 中常见的数据编码的形式，cv_bridge可以有选择的对颜色和深度信息进行转化。为了使用指定的特征编码，就有下面集中的编码形式：

mono8:  CV_8UC1， 灰度图像

mono16: CV_16UC1,16位灰度图像

bgr8: CV_8UC3,带有颜色信息并且颜色的顺序是BGR顺序

rgb8: CV_8UC3,带有颜色信息并且颜色的顺序是RGB顺序

bgra8: CV_8UC4, BGR的彩色图像，并且带alpha通道

rgba8: CV_8UC4,CV，RGB彩色图像，并且带alpha通道

注：这其中mono8和bgr8两种图像编码格式是大多数OpenCV的编码格式