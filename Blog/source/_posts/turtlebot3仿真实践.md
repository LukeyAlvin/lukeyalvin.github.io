---
title: turtlebot3仿真实践
tags: [ros,机器人仿真,gazebo,turtlebot3]
date: {{ date }}
comments: true
mathjax: true
categories: ROS

---

{%cq%}
    TurtleBot3 是一个小型，低成本，完全可编程，基于 ROS 的移动机器人。 它旨在用于教育，研究，产品原型和爱好应用的目的。TurtleBot3 的目标是大幅降低平台的尺寸和价格，而不会牺牲性能，功能和质量。由于提供了其他选项，如底盘，计算机和传感器，TurtleBot3 可以通过各种方式进行定制。TurtleBot3 意愿通过应用 SBC（单板计算机），深度传感器和 3D 打印的最新技术进步，成为创客运动的中心。

{%endcq%}

<!-- more -->

# 下载turtlebot3仿真包

- 环境准备

```bash
$ sudo apt-get install ros-noetic-turtlebot3 ros-noetic-turtlebot3-description ros-noetic-turtlebot3-gazebo ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3-slam ros-noetic-turtlebot3-teleop
```

以上会避免之后问题里出现的报错

```bash
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
```

- 问题:turtlebot3_msgs报错

```bash
$ cd ~/catkin_ws && catkin_make
Could not find a package configuration file provided by
"turtlebot3_msgs" with any of the following names:
 
turtlebot3_msgsConfig.cmake
turtlebot3_msgs-config.cmake
 
Add the installation prefix of "turtlebot3_msgs" to CMAKE_PREFIX_PATH or set "turtlebot3_msg_DIR" to a directory containing one of the above files.
```

解决方案:这是因为缺少相关的包

```bash
sudo apt-get install ros-noetic-turtlebot3-msgs
```

## Empty World

```bash
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

- 问题:Resource not found: turtlebot3_description

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-02_22-21.png"/>

解决方案:这种情况的发生是因为turtlebot3_gazebo包没有对`turtlebot3_description`声名依赖,你应该安装一下这个依赖

```bash
ros-noetic-turtlebot3-description
```

其他版本的ros可以尝试更换`noetic`为你当前的版本

参考链接:[**Error in "roslaunch turtlebot3_gazebo turtlebot3_world.launch "**](https://answers.ros.org/question/348299/error-in-roslaunch-turtlebot3_gazebo-turtlebot3_worldlaunch/)

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-02_22-30.png"/>

## TurtleBot3 World

```bash
$ export TURTLEBOT3_MODEL=waffle
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-02_22-34.png"/>

## TurtleBot3 House

```bash
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-02_23-16.png"/>

# SLAM仿真

在 Gazebo 模拟器中进行 SLAM 时，您可以在虚拟世界中选择或创建各种环境和机器人模型。 除了准备模拟环境而不是启动机器人之外，SLAM 仿真与实际的 TurtleBot3 的SLAM非常相似。

## 加载仿真环境

准备了三个 Gazebo 环境，但要使用 SLAM 创建地图，建议使用 **TurtleBot3 World** 或 **TurtleBot3 House**。
使用以下命令之一加载 Gazebo 环境。

在本指令中，将使用 TurtleBot3 World。
请在 `burger`、`waffle`、`waffle_pi` 中为 `TURTLEBOT3_MODEL` 参数使用正确的关键字。

```bash
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

## 运行SLAM节点

从远程 PC 使用 `Ctrl` + `Alt` + `T` 打开一个新终端并运行 SLAM 节点。 默认使用 Gmapping SLAM 方法。
请在 `burger`、`waffle`、`waffle_pi` 中为 `TURTLEBOT3_MODEL` 参数使用正确的关键字。

```bash
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

## 运行键盘控制节点

使用`Ctrl`+`Alt`+`T`从远程 PC 打开一个新终端，然后从远程 PC 运行远程操作节点。
请在 `burger`、`waffle`、`waffle_pi` 中为 `TURTLEBOT3_MODEL` 参数使用正确的关键字。

```bash
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

 Control Your TurtleBot3!
 ---------------------------
 Moving around:
        w
   a    s    d
        x

 w/x : increase/decrease linear velocity
 a/d : increase/decrease angular velocity
 space key, s : force stop

 CTRL-C to quit
```

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-02_23-50.png"/>

# 导航仿真

就像 Gazebo 模拟器中的 SLAM 一样，你可以在虚拟导航世界中选择或创建各种环境和机器人模型。 但是，在运行导航之前必须准备适当的地图。 除了准备仿真环境而不是制作机器人之外，仿真导航与实际的导航非常相似。

## 加载仿真环境

在前面的 SLAM 部分中，TurtleBot3 World 用于创建地图。 导航将使用相同的 Gazebo 环境。请在 burger、waffle、waffle_pi 中为 TURTLEBOT3_MODEL 参数使用正确的关键字。

```bash
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

## 运行导航节点

```bash
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

## 初始姿态估计

必须在运行导航之前执行初始姿态估计，因为此过程会初始化对导航至关重要的 AMCL 参数。 TurtleBot3 必须正确定位在地图上，LDS 传感器数据与显示的地图整齐地重叠。

单击 RViz 菜单中的 2D Pose Estimate 按钮。

单击实际机器人所在的地图，然后将绿色大箭头拖向机器人面向的方向。

重复步骤 1 和 2，直到 LDS 传感器数据覆盖在保存的地图上。

启动键盘遥操作节点，在地图上精确定位机器人。

```bash
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

