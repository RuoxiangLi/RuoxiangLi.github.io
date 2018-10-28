---
title: 当前开源SLAM方案一览
date: 2018-09-26 15:52:25
tags:
  - SLAM
mathjax: true
categories:
  - 机器人 
  - SLAM
  - 其他
---
---
这篇文章记录下目前流行的开源SLAM系统的方案。
<!--more--->
最近课题进展不顺利，目标不明确、思路不清晰、没有创新点，迷茫、不知所措。还是暂时停一下，花时间查阅一些资料，充实一下自己。

| 时间 |    开源方案    |    传感器形式     |    VO    | 稀疏\稠密 | 论文 |                           地址链接                           |
| :--: | :------------: | :---------------: | :------: | :-------: | :--: | :----------------------------------------------------------: |
| 2007 |    MonoSLAM    |       单目        |          |           | [1]  |       [Github](https://github.com/hanmekim/SceneLib2)        |
| 2007 |      PTAM      |       单目        |          |           | [2]  |     [Source Code]( http://www.robots.ox.ac.uk/~gk/PTAM/)     |
| 2015 |    ORB-SLAM    |     单目为主      | 特征点法 |   稀疏    | [3]  | [链接](http://webdiis.unizar.es/~raulmur/orbslam/)   [Github](https://github.com/raulmur/ORB_SLAM) |
| 2017 |   ORB-SLAM2    | 单目、双目、RGB-D | 特征点法 |   稀疏    | [4]  |        [Github](https://github.com/raulmur/ORB_SLAM2)        |
| 2014 |    LSD-SLAM    |     单目为主      |  直接法  |  半稠密   | [5]  | [Vision]( http://vision.in.tum.de/research/vslam/lsdslam)   [Github](<https://github.com/tum-vision/lsd_slam> ) |
| 2014 |      SVO       |       单目        |  直接法  |           | [6]  |         [Github](https://github.com/uzh-rpg/rpg_svo)         |
| 2014 |    RTAB-MAP    |    RGB-D/双目     |          |           | [7]  |        [Github](https://github.com/introlab/rtabmap )        |
| 2015 |     OKVIS      |     多目+IMU      |          |           | [8]  |         [Github](https://github.com/ethz-asl/okvis )         |
| 2015 |     ROVIO      |     单目+IMU      |          |           | [9]  |         [Github](https://github.com/ethz-asl/rovio)          |
| 2011 |      DTAM      |       RGB-D       |  直接法  |   稠密    | [10] |       [Github](https://github.com/anuranbaka/OpenDTAM)       |
| 2013 |      DVO       |       RGB-D       |          |           | [11] |       [Github](https://github.com/tum-vision/dvo_slam)       |
| 2014 |      DSO       |       单目        |          |           | [12] |         [Github](https://github.com/JakobEngel/dso)          |
| 2014 |   RGBD-SLAM2   |       RGB-D       |          |           | [13] |     [Github](https://github.com/felixendres/rgbdslam_v2)     |
| 2015 | Elastic Fusion |       RGB-D       |          |   稠密    | [14] |      [Github](https://github.com/mp3guy/ElasticFusion)       |
| 2011 |  Hector SLAM   |       激光        |          |           | [15] |           [wiki](http://wiki.ros.org/hector_slam)            |
| 2007 |    GMapping    |       激光        |          |           | [16] |             [wiki](http://wiki.ros.org/gmapping)             |
| 2015 |     OKVIS      |     多目+IMU      |          |           | [17] |         [Github](https://github.com/ethz-asl/ckvis)          |
| 2015 |     ROVIO      |     单目+IMU      |          |           | [18] | [Github](https://github.com/ethz-asl/rovio)  [Paper](http://dx.doi.org/10.3929/ethz-a-010566547) |
| 2011 | Kinetic Fusion |       RGB-D       |          |   稠密    | [19] |                                                              |
|      |   Kintinuous   |                   |          |           | [20] |                                                              |
|      | DynamicFusion  |                   |          |   稠密    | [21] |                                                              |
|      |   InfiniTAM    |                   |          |   稠密    | [22] |                                                              |
|      |      LSD       |    单目、双目     |  直接法  |  半稠密   | [23] | [Github](https://github.com/tum-vision/lsd_slam) [home](https://vision.in.tum.de/research/vslam/lsdslam) |

## 论文

[3] 

- [Raúl Mur-Artal](http://webdiis.unizar.es/~raulmur/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Juan D. Tardós](http://webdiis.unizar.es/~jdtardos/). ORB-SLAM: A Versatile and Accurate Monocular SLAM System.  IEEE Transactions on Robotics, vol. 31, no. 5, pp. 1147-1163, October 2015. [[pdf\]](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)
- [Raúl Mur-Artal](http://webdiis.unizar.es/~raulmur/) and [Juan D. Tardós](http://webdiis.unizar.es/~jdtardos/). Probabilistic Semi-Dense Mapping from Highly Accurate Feature-Based Monocular SLAM. Robotics: Science and Systems. Rome, Italy, July 2015. [[pdf\]](http://webdiis.unizar.es/~raulmur/MurTardosRSS15.pdf) [[poster\]](http://webdiis.unizar.es/~raulmur/MurTardosRSS15Poster.pdf)

[5] LSD-SLAM: Large-Scale Direct Monocular SLAM (J. Engel, T. Schöps, D. Cremers), In European Conference on Computer Vision (ECCV), 2014. [[bib\]](http://vision.in.tum.de/research/vslam/lsdslam?key=engel14eccv) [[pdf\]](http://vision.in.tum.de/_media/spezial/bib/engel14eccv.pdf) [[video\]](http://vision.in.tum.de/_media/spezial/bib/engel14eccv.mp4)

[10] R. A. Newcombe, S. Lovegrove, and A. J. Davison. Dtam: Dense tracking and mapping in real-time. In IEEE International Conference on Computer Vision (ICCV), pages 2320–2327, 2011. 1, 2, 3

[11]

- **Dense Visual SLAM for RGB-D Cameras** (C. Kerl, J. Sturm, D. Cremers), In Proc. of the Int. Conf. on Intelligent Robot Systems (IROS), 2013.
- **Robust Odometry Estimation for RGB-D Cameras** (C. Kerl, J. Sturm, D. Cremers), In Proc. of the IEEE Int. Conf. on Robotics and Automation (ICRA), 2013
- **Real-Time Visual Odometry from Dense RGB-D Images**  (F. Steinbruecker, J. Sturm, D. Cremers), In Workshop on Live Dense  Reconstruction with Moving Cameras at the Intl. Conf. on Computer Vision  (ICCV), 2011.

[12] Fast Semi-Direct Monocular Visual Odometry (ICRA 2014)

[13] "3D Mapping with an RGB-D Camera", F. Endres, J. Hess, J. Sturm, D. Cremers, W. Burgard, IEEE Transactions on Robotics, 2014.

\[14\] 

- **ElasticFusion: Real-Time Dense SLAM and Light Source Estimation**, *T. Whelan, R. F. Salas-Moreno, B. Glocker, A. J. Davison and S. Leutenegger*, IJRR '16
- **ElasticFusion: Dense SLAM Without A Pose Graph**, *T. Whelan, S. Leutenegger, R. F. Salas-Moreno, B. Glocker and A. J. Davison*, RSS '15

[15] A Flexible and Scalable SLAM System with Full 3D Motion Estimation

[16] Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters 

[17] Stefan Leutenegger, Simon Lynen, Michael Bosse, Roland Siegwart and Paul Timothy Furgale. [Keyframe-based visual–inertial odometry using nonlinear optimization](http://www.roboticsproceedings.org/rss09/p37.pdf). The International Journal of Robotics Research, 2015.

[20]

- [Real-time Large Scale Dense RGB-D SLAM with Volumetric Fusion](http://thomaswhelan.ie/Whelan14ijrr.pdf), T. Whelan, M. Kaess, H. Johannsson, M.F. Fallon, J. J. Leonard and J.B. McDonald, IJRR '14 
- [Kintinuous: Spatially Extended KinectFusion](http://thomaswhelan.ie/Whelan12rssw.pdf), T. Whelan, M. Kaess, M.F. Fallon, H. Johannsson, J. J. Leonard and J.B. McDonald, RSS RGB-D Workshop '12

[23]

- Reconstructing Street-Scenes in Real-Time From a Driving Car (V. Usenko, J. Engel, J. Stueckler, D. Cremers), In Proc. of the Int. Conference on 3D Vision (3DV), 2015.  [bib](https://vision.in.tum.de/research/vslam/lsdslam?key=usenko15_3drecon_stereolsdslam) [[pdf]](https://vision.in.tum.de/_media/spezial/bib/usenko15_3drecon_stereolsdslam.pdf)
- Large-Scale Direct SLAM for Omnidirectional Cameras (D. Caruso, J. Engel, D. Cremers),In International Conference on Intelligent Robots and Systems (IROS), 2015. [[bib\]](https://vision.in.tum.de/research/vslam/lsdslam?key=caruso2015_omni_lsdslam) [[pdf\]](https://vision.in.tum.de/_media/spezial/bib/caruso2015_omni_lsdslam.pdf) [[video\]](https://vision.in.tum.de/_media/spezial/bib/caruso2015_omni_lsdslam.mp4)
- Large-Scale Direct SLAM with Stereo Cameras (J. Engel, J. Stueckler, D. Cremers), In International Conference on Intelligent Robots and Systems (IROS), 2015.  [[bib\]](https://vision.in.tum.de/research/vslam/lsdslam?key=engel2015_stereo_lsdslam) [[pdf\]](https://vision.in.tum.de/_media/spezial/bib/engel2015_stereo_lsdslam.pdf) [[video\]](https://vision.in.tum.de/_media/spezial/bib/engel2015_stereo_lsdslam.mp4)
- Semi-Dense Visual Odometry for AR on a Smartphone (T. Schöps, J. Engel, D. Cremers), In International Symposium on Mixed and Augmented Reality, 2014.  [[bib\]](https://vision.in.tum.de/research/vslam/lsdslam?key=schoeps14ismar) [[pdf\]](https://vision.in.tum.de/_media/spezial/bib/schoeps14ismar.pdf) [[video\]](https://vision.in.tum.de/_media/spezial/bib/schoeps14ismar.mp4)
- Semi-Dense Visual Odometry for a Monocular Camera (J. Engel, J. Sturm, D. Cremers), In IEEE International Conference on Computer Vision (ICCV), 2013.  [[bib\]](https://vision.in.tum.de/research/vslam/lsdslam?key=engel2013iccv) [[pdf\]](https://vision.in.tum.de/_media/spezial/bib/engel2013iccv.pdf) [[video\]](https://vision.in.tum.de/_media/spezial/bib/engel2013iccv.avi)

## 参考资料

1. [当前的开源SLAM方案](https://www.cnblogs.com/Jessica-jie/p/7719359.html)
2. [【干货】15种SLAM方案详解](http://www.vrtuoluo.cn/8821.html)
3. [SLAM 综述](https://blog.csdn.net/darlingqiang/article/details/78901022)