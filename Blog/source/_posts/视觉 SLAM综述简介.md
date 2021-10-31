---
title: 视觉SLAM综述简介
date: {{ date }}
permalink:
categories: 文献阅读
tags: [文献阅读]
comments: true
---
{%cq%}
    本文选<权美香，朴松昊，李国． 视觉 ＳＬＡＭ 综述>一文,对整个论文进行梳理,让自己对整个SLAM的框架有一定的了解.
{%endcq%}
<!-- more -->
# 视觉SLAM综述简介

>参考文献：
>
>[1]权美香，朴松昊，李国． 视觉 ＳＬＡＭ 综述［Ｊ］． 智能系统学报， ２０１６， １１（６）： ７６８－７７６．

## 视觉SLAM方法介绍

### 视觉 SLAM的概念

视觉 SLAM指的是相机作为唯一的外部传感器，在进行自身定位的同时创建环境地图。 SLAM创建的地图
的好坏对之后自主的定位、路径规划以及壁障的性能起到一个决定性的作用。

### 视觉SLAM的分类

- 单目视觉 SLAM (仅用一个相机作为唯一外部传感器)
- 立体视觉 SLAM (使用多个相机作为传感器)
- RGB-D SLAM (基于单目相机与红外传感器结合构成的传感器)

### 视觉SLAM的方法介绍

根据利用图像信息的不同

- 基于特征的 SLAM 方法
- direct SLAM 方法

#### 基于特征的 SLAM 方法

##### 简介

基于特征的 SLAM 方法指的是对输入的图像进行**特征点检测及提取**，并基于 ２⁃Ｄ 或 ３⁃Ｄ 的特征匹配**计算相机位姿及对环境**进行建图。如果对整幅图像进行处理，则计算复杂度太高，由于特征在保存图像重要信息的同时有效减少了计算量，从而被广泛使用。

##### 发展

###### 借助于**滤波器**实现的单目视觉SLAM

- 扩展卡尔曼滤波器（extended Kalman filter，EKF）

  **原理：**利用扩展卡尔曼滤波器来实现同时定位与地图创建，其主要思想是使用状态向量来存储相机位姿及地图点的三维坐标，利用概率密度函数来表示不确定性，从观测模型和递归的计算，最终获得更新的状态向量的均值和方差。

  **缺点：**由于EKF的引进，SLAM算法会有计算复杂度及由于线性化而带来的不确定性问题。

- 无迹卡尔曼滤波器（Unscented Kal-man filter，UKF）

  该方法虽然对不确定性有所改善，但同时也增加了计算复杂度。

- Rao-Blackwellized粒子滤波（Particle filter）

  该方法避免了线性化，且对相机的快速运动有一定的弹力，但是为了保证定位精度，则需要使用较多的粒子，从而大大提高了计算复杂度

###### 基于关键帧的单目视觉SLAM

- Parallel Tracking and Mapping for Small AR Workspaces（PTAM）

  该论文提出了一个简单、有效的提取关键帧的方法，且将定位和创建地图分为两个独立的任务，并在两个线程上进行。

- A Versatile and Accurate Monocular SLAM System

  在关键帧的基础上提出的一个单目视觉 SLAM 系统，将整个 SLAM 过程分为定位、创建地图、闭环 ３个线程，且对这 ３ 个任务使用相同的 ORB 特征，且引进本质图的概念来加速闭环校正过程。

###### 基于RGB-D数据的SLAM

微软公司推出的Kinect相机，能够同时获得图像信息及深度信息，从而简化了三维重建的过程，且由于价格便宜，基于RGB_D数据的SLAM得到了迅速的发展。

《RGB-D map-ping: using depth cameras for dense 3-D modeling of in-door environments》最早提出的使用
RGB-D相机对室内环境进行三维重建的方法，在彩色图像中提取 SHIF特征并在深度图像上查找相应的深度信息。然后使用RANSAC方法对3-D特征点进行匹配并计算出相应的刚体运动变换，再以此作为ICP (iterative closest point）的初始值来求出更精确的位姿。

![image-20211009165001747](https://gitee.com/LukeyAlvin/img_mk/raw/master/img/image-20211009165001747.png)

#### 直接的SLAM 方法

##### 简介

直接的SLAM方法指的是直接对**像素点的强度**进行操作，避免了特征点的提取，该方法能够使用图像的所有信息。此外，提供更多的环境几何信息，有助于对地图的后续使用。且对特征较少的环境有更高的准确性和鲁棒性。

##### 发展

- 《Real-time dense geometry from a handheld camera》

相机定位方法依赖图像的每个像素点，即用稠密的图像对准来进行自身定位，并构建出稠密的3-D地图。

- 《Semi-Dense Visual Odometry for a Monocular Camera 》

对当前图像构建半稠密inverse深度地图，并使用稠密图像配准（dense image alignment）法计算相机位姿。构建半稠密地图即估计图像中梯度较大的所有像素的深度值，该深度值被表示为高斯分布，且当新的图像到来时，该深度值被更新。

- 《Dense tracking and mapping in real-time》

对每个像素点进行概率的深度测量，有效降低了位姿估计的不确定性。

- 《Fast semi-direct monocular visual odometry 》

提出了一种半直接的单目视觉里程计方法，该方法相比于直接法不是对整幅图像进行直接匹配从而获得相机位姿，而是通过在整幅图像中提取的图像块来进行位姿的获取，这样能够增强算法的鲁棒性。

- 《Large-Scale Direct Monocular SLAM》

为了构建稠密的三维环境地图，Engel等提出了LSD-SLAM算法（large-scale direct SLAM），相比之前的直接的视觉里程计方法，该方法在估计高准确性的相机位姿的同时能够创建大规模的三维环境地图。

- 《Kinect-Fusion: Real-time dense surface mapping and tracking》27

提出了kinect融合的方法，该方法通过Kinect获取的深度图像对每帧图像中的每个像素进行最小化距离测量而获得相机位姿，且融合所有深度图像，从而获得全局地图信息。

- 《A dense map building approach from spherical RGBD images》28

使用图像像素点的光度信息和几何信息来构造误差函数，通过最小化误差函数而获得相机位姿，且地图问题被处理为位姿图表示。

- 《Dense visual SLAM for RGB-D cameras》29

这是较好的直接RGBD SLAM方法，该方法结合像素点的强度误差与深度误差作为误差函数，通过最小化代价函数，从而求出最优相机位姿，该过程由g20实现，并提出了基于熵的关键帧提取及闭环检方法，从而大大降低了路径的误差。

- 《Distinctive image features from scale-invariant keypoints》30

### 视觉 SLAM主要标志性成果

#### MonoSLAM  

> 《Mono-SLAM: real-time single camera SLAM》

Andrew Davison提出的第1个基于EKF方法的单目SLAM，能够达到实时但是不能确定漂移多少，能够在概率框架下在线创建稀疏地图。

#### DTAM

> 《DTAM: Dense tracking and mapping in real-time》

2011年提出的基于直接法的单目SLAM算法，该方法通过帧率的整幅图像对准来获得相对于稠密地图的相机的6个自由度位姿，能够在GPU上达到实时的效果。

#### PTAM

> 《Parallel Tracking and Mapping for Small AR Workspaces》

由Georg Klein提出的第1个用多线程处理SLAM的算法，将跟踪和建图分为两个单独的任务并在两个平行的线程进行处理。

#### Ki-nectFusion

> 《Kinect-Fusion: Real-time dense surface mapping and tracking》

第1个基于Kinect的能在GPU上实时构建稠密三维地图的算法，该方法仅使用Kinect相机获取的深度信息去计算传感器的位姿以及构建精确的环境3-D地图模型。

#### LSD-SLAM

> 《LSD-SLAM: Large-Scale Direct Monocular SLAM》

直接的单目SLAM方法，即直接对图像的像素点进行处理，相比于之前的基于直接法的单目视觉里程计，不仅能计算出自身的位姿，还能构建出全局的半稠密且精确的环境地图。其中的追踪方法，直接在sim3上进行操作，从而能够准确地检测尺度漂移，可在CPU上实时运行。

#### ORB_SLAM

> 《ORB-SLAM: A Versatile and Accurate Monocular SLAM  System》

2015年出的比较完整的基于关键帧的单目SLAM算法，将整个系统分为追踪、地图创建、闭环控制3个线程进行处理，且特征的提取与匹配、稀疏地图的创建、位置识别都是基于ORB特征，其定位精确度很高，且可以实时运行。

### SLAM的主要研究实验室

1）**苏黎世联邦理工学院的Autonomous System Lab**:该实验室在tango项目上与谷歌合作，负责视觉-
惯导的里程计，基于视觉的定位和深度重建算法。
2）**明尼苏达大学的Multiple Autonomous Robotic Systems Laboratory**，主要研究四轴飞行器导航，合作建图，基于地图的定位，半稠密地图创建等。
3）**慕尼黑理工大学的The Computer Vision Group**，主要研究基于图像的3-D重建，机器人视觉视觉SLAM等。

## 视觉 SLAM 关键性问题

### 特征检测与匹配

目前点特征使用最多,使用最多的点特征如下

- **SIFT（scale invariant feature transform）特征**

SIFT特征已发展10多年，且获得了巨大的成功。SIFT特征具有可辨别性，由于其描述符用高维向量（128维）表示，且具有旋转不变性、尺度不变性、放射变换不变性，对噪声和光照变化也有鲁棒性。[33-6]在视觉SLAM里使用了SIFT特征，但是由于SIFT特征的向量维数太高，导致时间复杂度高。

《Distinctive image features from scale-invariant keypoints》

- **SURT（speeded up robust features）[31]特征**

SURF特征具有尺度不变性、旋转不变性，且相对于SIFT特征的算法速度提高了3到7倍。在文献[37-39]SURF被作为视觉SLAM的特征提取方法，与SIFT特征相比，时间复杂度有所降低。对两幅图像的SIFT和SURF特征进行匹配时通常是计算两个特征向量之间的欧氏距离，并以此作为特征点的相似性判断度量。

《spee-ded up robust features》

- **ORB（oriented fast and rotated BRIEF）[32]特征。**

ORB特征是FAST特征检测算子与BRIEF描述符的结合，并在其基础上做了一些改进。ORB特征最大的优点是计算速度快，是SIFT特征的100倍，SURF特征的10倍，其原因是FAST特征检测速度就很快，再加上BRIEF描述符是二进制串，大大缩减了匹配速度，而且具有旋转不变性，但不具备尺度不变性。ORB特征匹配是以BRIEF二进制描述符的汉明距离为相似性度量的。

《An efficient alternative to SIFT or SURF》

在大量包含直线和曲线的环境下，使用点特征时，环境中很多信息都将被遗弃，为了弥补这个缺陷，从而也提出了**基于边特征的视觉SLAM**和**基于区域特征的视觉SLAM**方法。

> 基于边特征的视觉SLAM《Edge landmarks in monocular SLAM》《Improving the agility of keyframe-based SLAM 》

> 基于区域特征的视觉SLAM《Using superpixels in monocular SLAM》

### 关键帧的选择

帧对帧的对准方法会造成大的累积漂浮，由于位姿估计过程中总会产生误差。为了减少帧对帧的对准方法带来的误差，基于关键帧的SLAM方法被提出。

《Parallel Tracking and Mapping for Small AR Workspaces》

《ORB-SLAM: A Versatile and Accurate Monocular SLAM System》

> 提出满足以下全部条件时该帧作为关键帧插入到地图里：从上一个关键帧经过了n个帧；当前帧至少能看到n个地图点，位姿估计准确性较高。

《RGB-D Mapping: Using Depth Cameras for Dense 3-D Modeling of Indoor Environments》

> 当两幅图像看到的共同特征点数低于一定阈值时，创建一个新的关键帧。

《Dense visual SLAM for RGB-D cameras 》

> 提出了一种基于熵的相似性的选择关键帧的方法，由于简单的阈值不适用于不同的场景，对每一帧计算一个熵的相似性比，如果该值小于一个预先定义的阈值，则前一帧被选为新的关键帧，并插入地图里，该方法大大减少了位姿漂浮。

### 闭环检测（loop closing）方法

闭环检测及位置识别，判断当前位置是否是以前已访问过的环境区域。三维重建过程中必然会产生误差累积，实现闭环是消除的一种手段。在位置识别算法中，视觉是主要的传感器。

文献《A compari-son of loop closing techniques in monocular SLAM》对闭环检测方法进行了比较

且得出图像对图像的匹配性能**优于**地图对地图，图像对地图的匹配方法。

- 图像对图像《Tracking and mapping recognizable features》《Accelerated appearance-only SLAM》

  > 图像对图像的匹配方法中，词袋（bag of words）方法由于其有效性得到了广泛的应用
  >
  > 《Scalable Recognition with a Vocabulary Tree》

- 地图对地图《Unscented SLAM for large-scale outdoor environments 》

- 图像对地图《Probabilistic Lo-calization and Mapping in the Space of Appearance》

《Unified loop closing and recovery for real time monocular SLAM》

> 对重定位和闭环检测提出了统一的方法，它们使用基于16维的SIFT特征的词典方法不断地搜索已访问过的位置。

《Real-time loop detec-tion with bags of binary words 》《Appearance-only SLAM at large scale with FAB-MAP》

> 使用基于SURF描述符的词典方法去进行闭环检测SURF特征，SURF特征提取需要花费400 ms去进行。

《Distinctive Image features from scale-invariant keypoints》

> 使用SIFT特征执行全局定位，且用KD树来排列地图点。

《Bags of binary words for fast place recognition in image sequences》

> 提出了一种使用基于FAST特征检测与BRIEF二进制描述符词典，且添加了直接索引（direct index），直接索引的引入使得能够有效地获得图像之间的匹配点，从而加快闭环检测的几何验证。

《Fast relocalisation andloop closing in keyframe-based SLAM》

> 用基于ORB特征的词典方法进行位置识别，由于ORB特征具有旋转不变性且能处理尺度变化，该方法能识别位置从不同的视角。

《ORB-SLAM: A Versatile and Accurate Monocular SLAM System》

> 该文献的位置识别方法建于文献《Fast relocalisation andloop closing in keyframe-based SLAM》的主要思想上，即使用基于ORB特征的词典方法选出候选闭环，再通过相似性计算进行闭环的几何验证。

### 地图优化

#### 简介

对于一个在复杂且动态的环境下工作的机器人，3-D地图的快速生成是非常重要的，且创建的环境地图对之后的定位、路径规划及壁障的性能起到一个关键性的作用，从而精确的地图创建也是非常重要的。

#### 问题

闭环检测成功后，往地图里添加闭环约束，执行闭环校正。闭环问题可以描述为**大规模的光束平差法（bundle adjustment）问题**，即对相机位姿及所有的地图点3-D坐标进行优化，但是该优化计算复杂度太高，从而很难实现实时。

#### 优化算法

- 通过位姿图优化（pose graph optimization）方法来对闭环进行优化

顶点为相机位姿，边表示位姿之间相对变换的图称为位姿图，位姿图优化即将闭环误差沿着图进行分配，即均匀分配到图上的所有位姿上。图优化通常由图优化框架g2o（general graph optimization）《A general framework for graph optimization 》里的LM（leven-berg-marquardt）算法实现。

《Dense visual SLAM for RGB-D cameras》

> 提出的RGB-D SLAM算法的位姿图里每个边具有一个权重，从而在优化过程中，不确定性高的边比不确定性低的边需要变化更多去补偿误差，并在最后，对图里的每个顶点进行额外的闭环检测且重新优化整个图。

《Scale drift-aware large scale monocular SLAM》

> 在闭环校正步骤使用了位姿图优化技术去实现旋转，平移及尺度漂浮的有效校正。

《ORB-SLAM: A Versatile and Accurate Monocular SLAM System》

> 在闭环检测成功后构建了本质图，并对该图进行位姿图优化。本质图包含所有的关键帧，但相比于covisibility图，减少了关键帧之间的边约束。本质图包含生成树、闭环连接及covisibility图里权重较大的边。

## 视觉SLAM主要发展趋势与研究热点

### 多传感器融合

#### 简介

相机能够捕捉场景的丰富细节，而惯性测量单元（inertial measurement unit，IMU）有高的帧率且相对小的能够获得准确的短时间估计，这两个传感器能够相互互补，从而一起使用能够获得更好的结果。

#### 发展

最初的视觉与IMU结合的位姿估计是用**滤波方法**解决的，用IMU的测量值作为预测值，视觉的测量值用于更新。

《A multi-state con-straint Kalman filter for vision-aided inertial navigation》

> 提出了一种基于EKF的IMU与单目视觉的实时融合方法，提出一种测量模型能够表示一个静态特征被多个相机所观察时的几何约束，该测量模型是最优的且不需要在EKF的状态向量里包括特征的3-D坐标。

《A dual-layer estima-tor architecture for long-term localization》

> 将融合问题分为两个线程进行处理，连续图像之间的惯性测量和特征跟踪被局部地在第1个线程进行处理，提供高频率的位置估计，第2个线程包含一个间歇工作的光束法平差的迭代估计，能够减少线性误差的影响。许多结果都已证明在准确性上基于优化的视觉SLAM优于基于滤波的SLAM方法。

《Keyframe-based visual-inertial slam using nonlinear opti-mization》

> 将IMU的误差以全概率的形式融合到路标的重投影误差里，构成将被优化的联合非线性误差函数，其中通过关键帧来边缘化之前的状态去维持一个固定大小的优化窗口，保证实时操作。考虑到基于优化方法的视觉-惯导导航的计算复杂度问题，

参考https://www. google. com/a-tap/projecttango/.

> 通过预积分选出的关键帧之间的惯性测量来进行解决，预积分能够精确地概括数百个惯性测量到一个单独的相对运动约束，这个预积分的IMU模型能被完美地融合到视觉-惯性的因子图的框架下。该系统的实验结果表明该系统要比Google的Tango还要精确。

### SLAM与深度学习的结合

随着深度学习在计算机视觉领域的大成功，大家对深度学习在机器人领域的应用有很大的兴趣。SLAM是一个大系统，里面有很多子模块，例如闭环检测，立体匹配等，都可通过深度学习的使用来获得更优的结果。

#### 发展

《Stereo matching by training a convolutional neural network to compare image patches》

> 提出了一种基于深度学习的立体匹配方法，用卷积神经网络来学习小图像块间的相似性，该卷积神经网络输出的结果用于线性化立体匹配代价。

《On the per-formance of ConvNet features for place recognition》

> 通过整合局部敏感散列法和新的语义搜寻空间划分的优化技术，使用卷积神经网络和大的地图达到实时的位置识别。

《Exploring representation learning with CNNs for frame-to-frame egomotion estimation》

> 使用卷积神经网络去学习视觉里程计的最佳的视觉特征和最优的估计器。

《Modelling uncertainty in deep learning for camera relocalization》

> 提出了一种重定位系统，使用贝叶斯卷积神经网络从单个彩色图像计算出六个自由度的相机位姿及其不确定性。

## 视觉SLAM的优缺点分析

### 单目视觉SLAM

#### 优点

单目相机应用灵活、简单、价格低。

#### 缺点

单目视觉SLAM在每个时刻只能获取一张图像，且只能依靠获得的图像数据计算环境物体的方向信息，无法直接获得可靠的深度信息，从而初始地图创建及特征点的深度恢复都比较困难。

此外，尺度不确定性是单目SLAM的主要特点，它是主要的误差源之一，但是正是尺度不确定性才使得单目SLAM能够在大尺度和小尺度环境之间进行自由转换。

### 双目视觉SLAM

#### 优点

双目视觉SLAM利用外极线几何约束的原理去匹配左右两个相机的特征，从而能够在当前帧速率的条件下直接提取完整的特征数据，因而应用比较广泛，它直接解决了系统地图特征的初始化问题。

#### 缺点

但是系统设计比较复杂，系统成本比较高，且它的视角范围受到一定限制，不能够获取远处的场景，从而只能在一定的尺度范围内进行可靠的测量，从而缺乏灵活性。

### RGBD SLAM

#### 优点

深度相机在获得彩色图像的同时获得深度图像，从而方便获得深度信息，且能够获得稠密的地图

#### 缺点

成本高，体积大，有效探测距离太短，从而可应用环境很有限。

## 结束

为了弥补视觉信息的不足，视觉传感器可以与惯性传感器（IMU）、激光等传感器融合，通过传感器之间的互补获得更加理想的结果。此外，为了能在实际环境中进行应用，SLAM的鲁棒性需要很高，从而足够在各种复杂环境下进行准确的处理，SLAM的计算复杂度也不能太高，从而达到实时效果。

