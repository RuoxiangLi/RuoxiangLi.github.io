---
title: Event-based特征追踪评估方法
date: 2019-06-13 13:55:04
tags:
  - Event-based Feature
  - Event Camera
categories: 
  - 机器人
  - SLAM
  - Event Camera
copyright: true
---
---

记录UZH机器感知实验室的一个Github项目学习过程，该项目目的是对Event-based特征追踪算法进行评估，包括轨迹的显示、误差、追踪时间的统计评测等。
<!--more--->

Github项目地址：https://github.com/uzh-rpg/rpg_feature_tracking_analysis

## 基于Event的特征追踪算法评估工作流程

### 评估特征追踪轨迹Ground Truth的两个方法

- 基于同步图像帧和event的KLT特征追踪
- 基于路标点反投影的特征追踪

### 算法功能

- 使用上述方法之一评估用户的特征追踪方法
- 生成随时间变化的特征追踪误差图（可用于论文插图）
- 生成特征追踪的三维时空图
- 生成特征追踪视频
- 将用户的特征追踪方法与其他方法进行比较
- 生成单个特征的跟踪误差图

### 工作原理

特征追踪评估两个步骤：

- Ground Truth Tracker初始化
- Tracking

基于KLT追踪方法获取Ground Truth，该算法的输入为测试数据集图像帧序列（以rosbag文件的形式，rostopic为`/dvs/image_raw`），算法执行步骤：

- 对于每个特征轨迹，确定其初始时间
- 在初始化时，或者初始化之后找到第一帧灰度图像
- 特征的x、y位置坐标插值到轨迹中，对应当前帧的时间戳
- KLT方法追踪该特征，直到该特征丢失；对于每一帧图像都要更新模板

在阅读代码后，总结算法执行流程如下：

 - 对于追踪算法得到的特征轨迹，提取每一个特征轨迹的第一个特征的时间戳
 - 根据图像帧时间戳，找到与初始特征时间戳距离最近的图像帧，时间差不超过1e-5，否则选择比初始特征时间戳大的最小时间戳对应的图像
 - 一维线性插值找到图像帧时间戳时刻对应的特征点的坐标，[id, t_image, x_interp, y_interp]组成元组，构成一个图像帧中的特征
 - 收集到所有的初始图像帧特征，保存留作备用
 - 找到初始图像帧特征中的时间戳最小值，选择该值1e-4范围内的所有特征作为初始活跃特征
 - 使用LK光流法在前两帧图像中追踪这些特征
 - 追踪到的特征作为新的特征与收集到的所有初始特征中的部分特征进行合并，参与合并的部分初始特征是第二帧图像周围的特征，然后在第三帧图像中追踪合并的特征
 - 重复上述过程，直到所有图像帧使用完毕

基于Ground Truth的重投影，算法执行步骤：

- 对于每个特征轨迹，确定初始时间和位置
- 使用插值方法，确定每次初始化时的深度和位姿
- 使用位姿、深度、相机标定参数，back-projected特征
- 对于之后的每个位姿，将路标点重投影至图像帧中，生成特征轨迹
- 超出图像帧平面范围的特征将被丢弃

基于追踪的重投影过程不支持相机的畸变。

## 数据输入

需要使用`.txt`文件提供特征轨迹数据，内容包括特征ID（追踪到的属于同一个特征的event使用相同的ID）、特征位置坐标、特征时间戳。如下所示：

```yaml
# feature_id timestamp x y
25 1.403636580013555527e+09 125.827 15.615 
13 1.403636580015467890e+09 20.453 90.142 
...
```

用于Ground Truth生成的图像、位姿、深度图、相机参数必须以rosbag的格式提供。各数据的信息格式如下：

- images: `sensor_msgs/Image`
- poses: `geometry_msgs/PoseStamped`
- depth maps: `sensor_msgs/Image`-`CV_32F`

有关ros话题和rosbag文件信息，使用`dataset.yaml`文件提供。

```yaml
type: bag
name: relative/path/to/ros.bag  # relative path of bag

# For KLT based tracking 
image_topic: /dvs/image_raw  

# For reprojection based tracking
depth_map_topic: /dvs/depthmap
pose_topic: /dvs/pose
camera_info_topic: /dvs/camera_info
```

### 执行命令

#### 测试特征追踪算法

需要进行如下步骤：

- `--file /path/to/tracks.txt`：给定用户自己的特征追踪算法生成的特征追踪结果

- `--dataset /path/to/your_dataset.yaml`：给定测试数据集的配置文件，文件内容如下

  ```yaml
  type: "bag"
  name: "your_dataset.bag"  		# 测试数据集rosbag文件名
  image_topic: "/dvs/image_raw"	# rosbag文件图像相关话题（ETH数据集rosbag图像话题都是这个）
  ```

- `--root /path/to/bags`：测试数据所在路径（到测试数据集rosbag文件的父目录）

测试执行命令举例：

1. 测试1：`python evaluate_tracks.py --error_threshold 10 --tracker_type KLT --file ./example/tracks/bicycles_gehrig_eccv_18/tracks.txt --dataset ./example/dataset_params/bicycles.yaml --root ./example/bags --plot_3d --plot_errors --video_preview`
2. 测试2：`python evaluate_tracks.py --error_threshold 10 --tracker_type KLT --file ./example/tracks/bicycles_kueng_iros_16/tracks.txt --dataset ./example/dataset_params/bicycles.yaml --root ./example/bags --plot_3d --plot_errors --video_preview`

执行命令解释：

```shell
python evaluate_tracks.py 		# 评估用户特征追踪算法
		--error_threshold 10 	# 用在可视化和画图时的阈值，追踪误差超过该阈值的特征将被丢弃
		--tracker_type KLT 		# 特征ground truth追踪方法
		--file ./example/tracks/bicycles_gehrig_eccv_18/tracks.txt # 用户特征追踪算法得到的特征轨迹
		--dataset ./example/dataset_params/bicycles.yaml # 保存测试数据集的类型、名称、ros话题
		--root ./example/bags 	# 测试数据集保存路径
		--plot_3d 
		--plot_errors 
		--video_preview
```

#### 对比特征追踪方法

测试执行命令：

1. 测试1：`python compare_tracks.py --error_threshold 10 --root ./example/tracks/ --config ./example/comparison_params/bicycles.yaml --results_directory ./example/comparison_results/bicycles_gehrig_eccv_18_kueng_iros_16`

执行命令解释：

```shell
python compare_tracks.py 
		--error_threshold 10 
		--root ./example/tracks/ 
		--config ./example/comparison_params/bicycles.yaml 
		--results_directory ./example/comparison_results
```
