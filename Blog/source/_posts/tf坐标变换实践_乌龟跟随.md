---
title: tf坐标综合实践_乌龟跟随
tags: [ros,tf坐标变换,turtlesim]
date: 2021-11-01 10:44:49
comments: true
mathjax: true
categories: ROS
---

{%cq%}
    案例描述:程序启动之初: 产生两只乌龟，中间的乌龟(A) 和 左下乌龟(B), B 会自动运行至A的位置，并且键盘控制时，只是控制 A 的运动，但是 B 可以跟随 A 运行
{%endcq%}
<!-- more -->

**案例分析:** 乌龟跟随实现的核心，是乌龟A和B都要发布相对世界坐标系的坐标信息，然后，订阅到该信息需要转换获取A相对于B坐标系的信息，最后，再生成速度信息，并控制B运动。

1. 启动乌龟显示节点
2. 在乌龟显示窗体中生成一只新的乌龟(需要使用服务)
3. 编写两只乌龟发布坐标信息的节点
4. 编写订阅节点订阅坐标信息并生成新的相对关系生成速度信息

# 命令实现

```bash
#1.安装
$ sudo apt-get install ros-noetic-ros-tutorials ros-noetic-geometry-tutorials ros-noetic-rviz ros-noetic-rosbash ros-noetic-rqt-tf-tree
#2.运行
$ roslaunch turtle_tf turtle_tf_demo.launch
```



# C++代码实现

## 启动一只乌龟

首先,我们知道启动乌龟及其键盘控制节点的命令有三个,我们可以使用下面的launch文件代替:

```xml
<launch>
    <node name="gui" pkg="turtlesim"  type="turtlesim_node" />
    <node name="key" pkg="turtlesim"  type="turtle_teleop_key" />
</launch>
```

## 生成第二只乌龟

其次,启动一只乌龟之后,我们可以查看一下服务:

```bash
alvin@ros:~$ rosservice list
/clear
/gui/get_loggers
/gui/set_logger_level
/key/get_loggers
/key/set_logger_level
/kill
/reset
/rosout/get_loggers
/rosout/set_logger_level
/spawn
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
```

因此,生成第二只乌龟调用的是服务来创建,话题是`/spawn`

```bash
alvin@ros:~$ rosservice call /spawn "x: 0.0
y: 0.0
theta: 0.0
name: ''" 
```

这是使用命令调用服务来创建一只新的乌龟:乌龟的位置为:(x,y),乌龟头的朝向为theta弧度,乌龟的名字是"name"

那么,如何使用代码实现呢?

```c++
#include "ros/ros.h"
#include "turtlesim/Spawn.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"spawn_client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");
    client.waitForExistence();
    turtlesim::Spawn spawn;
    spawn.request.x = 2;
    spawn.request.y = 2;
    spawn.request.theta = 1.57;
    spawn.request.name = "turtle2";

    client.call(spawn); 
    ros::spin();
    return 0;
}
```

这里`client.call(spawn);`返回的是布尔,我们也可以接收一下,去判断是不是成功的生成了一只乌龟.

```c++
setlocale(LC_ALL,"");
bool flag = client.call(spawn); 
    if (flag)
    {
        ROS_INFO("新乌龟已经生成了!");
    }else{
        ROS_INFO("生成失败了....");
    }
```

CMakeLists.txt添加相关命令

```cmake
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(turtlesim_follow_spawn src/turtlesim_follow_spawn.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(turtlesim_follow_spawn ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(turtlesim_follow_spawn
  ${catkin_LIBRARIES}
)
```

此时你可以在运行第一步的launch文件的基础上执行以下命令,测试是否成功生成乌龟

```bash
alvin@ros:~/catkin_ws$ rosrun tf_pratice turtlesim_follow_spawn 
[ INFO] [1635770123.695878217]: 新乌龟已经生成了!
```

## 发布两只乌龟的坐标系信息

可以订阅乌龟的位姿信息，然后再转换成坐标信息，两只乌龟的实现逻辑相同，只是订阅的话题名称，生成的坐标信息等稍有差异，可以将差异部分通过参数传入:

- 该节点需要启动两次

- 每次启动时都需要传入乌龟节点名称(第一次是 turtle1 第二次是 turtle2)

```c++
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

//保存乌龟名称
std::string turtle_name;

void doPose(const turtlesim::Pose::ConstPtr &pose)
{
    //创建一个广播对象
    static tf2_ros::TransformBroadcaster broadcaster;

    //坐标点解析
    geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id = "/world";
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = turtle_name.c_str();
    tfs.transform.translation.x = pose->x;
    tfs.transform.translation.y = pose->y;
    tfs.transform.translation.z = 0.0;

    //将欧拉角转换成四元数
    tf2::Quaternion qnt;
    qnt.setRPY(0,0,pose->theta);
    tfs.transform.rotation.x = qnt.getX();
    tfs.transform.rotation.y = qnt.getY();
    tfs.transform.rotation.z = qnt.getZ();
    tfs.transform.rotation.w = qnt.getW();

    //发布坐标信息
    broadcaster.sendTransform(tfs);
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"turtle_pose_pub");
    ros::NodeHandle nh;
    //解析传入的命名空间
    if (argc != 2)
    {
        ROS_ERROR("请传入正确的参数");
    } else {
        turtle_name = argv[1];
        ROS_INFO("乌龟 %s 坐标发送启动",turtle_name.c_str());
    }

    //订阅乌龟的位姿信息
    ros::Subscriber sub = nh.subscribe<turtlesim::Pose>(turtle_name + "/pose",1000,doPose);
    ros::spin();
    return 0;
}
```

CMakeLists.txt添加相关命令

```cmake
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(turtlesim_follow_pub src/turtlesim_follow_pub.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(turtlesim_follow_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(turtlesim_follow_pub
  ${catkin_LIBRARIES}
)
```

综上,我们可以更改原本的launch文件,使用该launch文件完成以上三步

```xml
<launch>
    <!-- 启动第一只乌龟 -->
    <node name="turtle1_gui" pkg="turtlesim"  type="turtlesim_node" />
    <node name="turtle1_key" pkg="turtlesim"  type="turtle_teleop_key" />
    <!-- 产生第二只乌龟 -->
    <node name="turtle2_spaw" pkg="tf_pratice" type="turtlesim_follow_spawn" />
    <!-- 启动两个发布节点 -->
    <node name="caster1" pkg="tf_pratice" type="turtlesim_follow_pub" args="turtle1"/>
    <node name="caster2" pkg="tf_pratice" type="turtlesim_follow_pub" args="turtle2"/>
</launch>
```

我们查看下一下发布的话题

```bash
alvin@ros:~$ rostopic list 
/rosout
/rosout_agg
/tf
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
/turtle2/cmd_vel
/turtle2/color_sensor
/turtle2/pose
```

## 订阅:解析坐标信息并生成速度信息

现在我们已经获得两个乌龟的坐标系,以及相对于世界的坐标关系,可以在rviz内查看,现在需要做的是怎么才能让生成的turtle2跟随turtle1运动.

首先,需要获取 turtle1 相对 turtle2 的坐标信息,这样turtle2就可以找到turtle1.

然后,turtle需要发布速度信息,根据数学计算去发布新的速度指令,进而实现乌龟跟随

```c++
#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    //创建监听对象
    ros::init(argc,argv,"turtle_pose_sub");
    ros::NodeHandle nh;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    //需要创建发布 /turtle2/cmd_vel 的 publisher 对象
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel",1000);
	//注意这里的发布频率必须在10以上
    ros::Rate rate(100);
    while (ros::ok())
    {
        try
        {
            //先获取 turtle1 相对 turtle2 的坐标信息
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("turtle2","turtle1",ros::Time(0));

            //根据坐标信息生成速度信息 -- geometry_msgs/Twist.h
            geometry_msgs::Twist twist;
            twist.linear.x = 0.5 * sqrt(pow(tfs.transform.translation.x,2) 
                                        + pow(tfs.transform.translation.y,2));
            twist.angular.z = 4 * atan2(tfs.transform.translation.y,tfs.transform.translation.x);

            //发布速度信息 -- 需要提前创建 publish 对象
            pub.publish(twist);
        }
        catch(const std::exception& e)
        {
            ROS_INFO("发生异常:%s",e.what());
        }
        

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
```

CMakeLists.txt添加相关命令

```cmake
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(turtlesim_follow_sub src/turtlesim_follow_sub.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(turtlesim_follow_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(turtlesim_follow_sub
  ${catkin_LIBRARIES}
)
```

综上,我们可以更改原本的launch文件,使用该launch文件完成以上四步

```xml
<launch>
    <!-- 启动第一只乌龟 -->
    <node name="turtle1_gui" pkg="turtlesim"  type="turtlesim_node" />
    <node name="turtle1_key" pkg="turtlesim"  type="turtle_teleop_key" />
    <!-- 产生第二只乌龟 -->
    <node name="turtle2_spaw" pkg="tf_pratice" type="turtlesim_follow_spawn" />
    <!-- 启动两个发布节点 -->
    <node name="caster1" pkg="tf_pratice" type="turtlesim_follow_pub" args="turtle1"/>
    <node name="caster2" pkg="tf_pratice" type="turtlesim_follow_pub" args="turtle2"/>
    <!-- 订阅速度信息节点 -->
    <node name="listener" pkg="tf_pratice" type="turtlesim_follow_sub" />
</launch>
```

此时就可以实现跟随了
