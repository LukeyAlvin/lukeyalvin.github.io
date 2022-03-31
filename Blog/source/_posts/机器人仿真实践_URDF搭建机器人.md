---
title: 机器人仿真实践_URDF搭建机器人
tags: [ros,机器人仿真,URDF,rviz]
date: {{ date }}
comments: true
mathjax: true
categories: ROS
---

{%cq%}
    案例描述:创建一个四轮圆柱状机器人模型，机器人参数如下,底盘为圆柱状，半径 10cm，高 8cm，四轮由两个驱动轮和两个万向支撑轮组成，两个驱动轮半径为 3.25cm,轮胎宽度1.5cm，两个万向轮为球状，半径 0.75cm，底盘离地间距为 1.5cm(与万向轮直径一致)
{%endcq%}
<!-- more -->

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-02_13-46_RVIZ.png" >

**实现流程:**

创建机器人模型可以分步骤实现

1. 新建 urdf 文件，并与 launch 文件集成
2. 搭建底盘
3. 在底盘上添加两个驱动轮
4. 在底盘上添加两个万向轮

# 新建urdf以及launch文件

urdf 文件:基本实现

```xml
<robot name="mycar">
    <!-- 设置 base_footprint  -->
    <link name="base_footprint">
        <visual>
            <geometry>
                <sphere radius="0.001" />
            </geometry>
        </visual>
    </link>

    <!-- 添加底盘 -->


    <!-- 添加驱动轮 -->


    <!-- 添加万向轮(支撑轮) -->

</robot>
```

>robot
>
>> urdf 中为了保证 xml 语法的完整性，使用了`robot`标签作为根标签，所有的 link 和 joint 以及其他标签都必须包含在 robot 标签内,在该标签内可以通过 name 属性设置机器人模型的名称\
>
>name: 
>
>> 指定机器人模型的名称
>
>link
>
>>  urdf 中的 link 标签用于描述机器人某个部件(也即刚体部分)的外观和物理属性，比如: 机器人底座、轮子、激光雷达、摄像头...每一个部件都对应一个 link, 在 link 标签内，可以设计该部件的形状、尺寸、颜色、惯性矩阵、碰撞参数等一系列属性
>>
>> > - name ---> 为连杆命名
>> > - visual ---> 描述外观(对应的数据是可视的) 
>> >
>> > - - **geometry 设置连杆的形状** 
>> >
>> > - - - 标签1: box(盒状) 
>> >
>> > - - - - 属性:size=长(x) 宽(y) 高(z)
>> >
>> > - - - 标签2: cylinder(圆柱) 
>> >
>> > - - - - 属性:radius=半径 length=高度
>> >
>> > - - - 标签3: sphere(球体) 
>> >
>> > - - - - 属性:radius=半径
>> >
>> > - - - 标签4: mesh(为连杆添加皮肤) 
>> >
>> > - - - - 属性: filename=资源路径(格式:**package:////文件**)
>> >
>> > - - **origin 设置偏移量与倾斜弧度** 
>> >
>> > - - - 属性1: xyz=x偏移 y便宜 z偏移
>> >
>> > - - - 属性2: rpy=x翻滚 y俯仰 z偏航 (单位是弧度)
>> >
>> > - - **metrial 设置材料属性(颜色)** 
>> >
>> > - - - 属性: name
>> >
>> > - - - 标签: color 
>> >
>> > - - - - 属性: rgba=红绿蓝权重值与透明度 (每个权重值以及透明度取值[0,1])
>> >
>> > - collision ---> 连杆的碰撞属性
>> >
>> > - Inertial ---> 连杆的惯性矩阵
>
>

launch 文件:

```xml
<launch>
    <!-- 将 urdf 文件内容设置进参数服务器 -->
   <!-- 将 urdf 文件内容设置进参数服务器 -->
    <param name="robot_description" 
           textfile="$(find robot_simlink_rviz)/urdf/robot_urdf.urdf" />
    <!-- 启动 rivz -->
    <node pkg="rviz" type="rviz" name="rviz_test" />

    <!-- 启动机器人状态和关节状态发布节点 -->
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />
    <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" />

</launch>
```

# 底盘搭建

```xml
<!-- 
        参数
            形状:圆柱 
            半径:10     cm 
            高度:8      cm 
            离地:1.5    cm

    -->
    <link name="base_link">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.08" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="yellow">
                <color rgba="0.8 0.3 0.1 0.5" />
            </material>
        </visual>
    </link>

    <joint name="base_link2base_footprint" type="fixed">
        <parent link="base_footprint" />
        <child link="base_link"/>
        <origin xyz="0 0 0.055" />
    </joint>
```

> joint
>
> urdf 中的 joint 标签用于描述机器人关节的运动学和动力学属性，还可以指定关节运动的安全极限，机器人的两个部件(分别称之为 parent link 与 child link)以"关节"的形式相连接，不同的关节有不同的运动形式: 旋转、滑动、固定、旋转速度、旋转角度限制....,比如:安装在底座上的轮子可以360度旋转，而摄像头则可能是完全固定在底座上。joint标签对应的数据在模型中是不可见的
>
> 属性
>
> > - name ---> 为关节命名
> >
> > - type ---> 关节运动形式 
> >
> > - - continuous: 旋转关节，可以绕单轴无限旋转
> >
> > - - revolute: 旋转关节，类似于 continues,但是有旋转角度限制
> >
> > - - prismatic: 滑动关节，沿某一轴线移动的关节，有位置极限
> >
> > - - planer: 平面关节，允许在平面正交方向上平移或旋转
> >
> > - - floating: 浮动关节，允许进行平移、旋转运动
> >
> > - - fixed: 固定关节，不允许运动的特殊关节
>
> 子标签
>
> > -  parent(必需的)
> >   parent link的名字是一个强制的属性： 
> >
> > - - link:父级连杆的名字，是这个link在机器人结构树中的名字。
> >
> > -  child(必需的)
> >   child link的名字是一个强制的属性： 
> >
> > - - link:子级连杆的名字，是这个link在机器人结构树中的名字。
> >
> > -  origin 
> >
> > - - 属性: xyz=各轴线上的偏移量 rpy=各轴线上的偏移弧度。
> >
> > -  axis 
> >
> > - - 属性: xyz用于设置围绕哪个关节轴运动。

# 添加驱动轮

```xml
<!-- 添加驱动轮 -->
    <!--
        驱动轮是侧翻的圆柱
        参数
            半径: 3.25 cm
            宽度: 1.5  cm
            颜色: 黑色
        关节设置:
            x = 0
            y = 底盘的半径 + 轮胎宽度 / 2
            z = 离地间距 + 底盘长度 / 2 - 轮胎半径 = 1.5 + 4 - 3.25 = 2.25(cm)
            axis = 0 1 0
    -->
    <link name="left_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.0325" length="0.015" />
            </geometry>
            <origin xyz="0 0 0" rpy="1.5705 0 0" />
            <material name="black">
                <color rgba="0.0 0.0 0.0 1.0" />
            </material>
        </visual>

    </link>

    <joint name="left_wheel2base_link" type="continuous">
        <parent link="base_link" />
        <child link="left_wheel" />
        <origin xyz="0 0.1 -0.0225" />
        <axis xyz="0 1 0" />
    </joint>


    <link name="right_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.0325" length="0.015" />
            </geometry>
            <origin xyz="0 0 0" rpy="1.5705 0 0" />
            <material name="black">
                <color rgba="0.0 0.0 0.0 1.0" />
            </material>
        </visual>

    </link>

    <joint name="right_wheel2base_link" type="continuous">
        <parent link="base_link" />
        <child link="right_wheel" />
        <origin xyz="0 -0.1 -0.0225" />
        <axis xyz="0 1 0" />
    </joint>
```



# 添加万向轮



```xml
<!-- 添加万向轮(支撑轮) -->
    <!--
        参数
            形状: 球体
            半径: 0.75 cm
            颜色: 黑色

        关节设置:
            x = 自定义(底盘半径 - 万向轮半径) = 0.1 - 0.0075 = 0.0925(cm)
            y = 0
            z = 底盘长度 / 2 + 离地间距 / 2 = 0.08 / 2 + 0.015 / 2 = 0.0475 
            axis= 1 1 1

    -->
    <link name="front_wheel">
        <visual>
            <geometry>
                <sphere radius="0.0075" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="black">
                <color rgba="0.0 0.0 0.0 1.0" />
            </material>
        </visual>
    </link>

    <joint name="front_wheel2base_link" type="continuous">
        <parent link="base_link" />
        <child link="front_wheel" />
        <origin xyz="0.0925 0 -0.0475" />
        <axis xyz="1 1 1" />
    </joint>

    <link name="back_wheel">
        <visual>
            <geometry>
                <sphere radius="0.0075" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="black">
                <color rgba="0.0 0.0 0.0 1.0" />
            </material>
        </visual>
    </link>

    <joint name="back_wheel2base_link" type="continuous">
        <parent link="base_link" />
        <child link="back_wheel" />
        <origin xyz="-0.0925 0 -0.0475" />
        <axis xyz="1 1 1" />
    </joint>
```

# 执行launch文件

```bash
alvin@ros:~/catkin_ws$ roslaunch robot_simlink_rviz robot_urdf.launch 
... logging to /home/alvin/.ros/log/429ebe1e-3ba0-11ec-9e19-e56937336ffc/roslaunch-ros-113111.log
.....
```

添加坐标系和机器人模型,然后就可以看到搭建的小车模型.

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-02_13-45_RVIZ.png"/>

我们也可以保存生成的rviz文件,以便二次打开反复添加参数,File/save config as...

修改launch文件

```xml
<launch>
    <!-- 将 urdf 文件内容设置进参数服务器 -->
    <param name="robot_description" textfile="$(find robot_simlink_rviz)/urdf/robot_urdf.urdf" />

    <!-- 启动 rivz -->
    <node pkg="rviz" type="rviz" name="rviz_test" args="-d $(find robot_simlink_rviz)/config/robot_urdf.rviz" />

    <!-- 启动机器人状态和关节状态发布节点 -->
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />
    <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" />

</launch>
```

**思考:**

- 为什么使用foot_print?

>因为默认情况下: 底盘的中心点位于地图原点上，所以会导致机器人模型是半沉到地下的情况产生，可以使用的优化策略，将初始 link 设置为一个尺寸极小的 link(比如半径为 0.001m 的球体，或边长为 0.001m 的立方体)，然后再在初始 link 上添加底盘等刚体，这样实现，虽然仍然存在初始link半沉的现象，但是基本可以忽略了。这个初始 link 一般称之为 base_footprint

- 上述代码实现存在什么问题吗？比如复用性！
