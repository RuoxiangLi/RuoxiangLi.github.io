---
title: Tensorflow学习笔记
date: 2019-06-13 13:55:22
tags:
  - Tensorflow
categories:
  - 深度学习
  - Tensorflow
mathjax: true
copyright: true
---
---

记录学习Tensorflow过程中的一些重要内容。
<!--more--->

## 卷积神经网络



## 图像数据处理

虽然，复杂的预处理过程会减慢整个训练过程。但，通过对图像的预处理，可以尽量避免模型收到无关因素的影响。在大部分图像识别问题中，通过图像预处理过程可以提高模型的准确率。Tensorflow中的图像处理函数：

- 图像解码处理：`tf.image.decode_jpeg`、`tf.image.decode_png`
- 图像编码处理：`tf.image.encode_jpeg`、`tf.image.encode_png`
- 图像大小调整：`tf.image.resize_images`，可以选择双线性插值、最近邻居法、双三插值法、面积插值法
- 图像数据转化为实数类型：`tf.image.convert_image_dtype`
- 图像裁剪或填充：`tf.image.resize_image_with_crop_or_pad`
- 图像大小按比例调整：`tf.image.central_crop`
- 图像翻转：`tf.image.flip_up_down`、`tf.image.flip_left_right`、`tf.image.random_flip_up_down`、`tf.image.random_flip_left_right`
- 图像色彩调整：`tf.image.adjust_brightness`、`tf.image.random_brightness`
- 图像对比度调整：`tf.image.adjust_contrast`、`tf.image.random_contrast`
- 图像色相调整：`tf.image.adjust_hue`、`tf.image.random_hue`
- 图像饱和度调整：`tf.image.adjust_saturation`、`tf.image.random_saturation`

### 队列

Tensorflow提供`FIFOQueue`、`RandomShuffleQueue`两种队列，前者是先进先出队列，后者是随机队列，即每次出队列操作得到的是当前队列所有元素中随机选择的一个元素。队列的作用总结如下：

- 一种数据结构
- 也是异步计算张量取值的一个重要机制，如多线程可同时向一个队列中写元素，或同时读取一个队列中的元素。

