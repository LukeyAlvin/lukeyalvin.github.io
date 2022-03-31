---
title: tf坐标变换实践_动态坐标变换
tags: [ros,tf坐标变换,turtlesim]
date: 2021-11-01 10:44:42
comments: true
mathjax: true
categories: ROS
---

{%cq%}
    案例描述:启动 turtlesim_node,该节点中窗体有一个世界坐标系(左下角为坐标系原点)，乌龟是另一个坐标系，键盘控制乌龟运动，将两个坐标系的相对位置动态发布。通俗的说,本案例就是将相对于乌龟位姿的偏移量为(1,1,0)的坐标点动态转换成世界坐标系.
{%endcq%}
<!-- more -->

**案例分析:**

1. 乌龟本身不但可以看作坐标系，也是世界坐标系中的一个坐标点
2. 订阅 turtle1/pose,可以获取乌龟在世界坐标系的 x坐标、y坐标、偏移量以及线速度和角速度
3. 将 pose 信息转换成 坐标系相对信息并发布

**实际案例:**

这个案例放在实际中的应用可以理解为这样,世界坐标系看成大地坐标,将乌龟看成是小车的地盘的坐标系,将将对于乌龟偏移量为(1,1,0)的点看成是小车的雷达,我们如何将不停运动的雷达的坐标转换成世界坐标系下的坐标呢?

> 显然,雷达对于小车的偏移量是始终不变的,所以我们需要做的就是将雷达在小车坐标系下的坐标转换成世界坐标;

# C++代码实现

## 发布者实现

```c++
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

void doPose(const turtlesim::Pose::ConstPtr &pose)
{
    //创建发布者对象
    static tf2_ros::TransformBroadcaster broadcaster;

    //坐标信息
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = "world";
    tfs.child_frame_id = "turtle1";
    tfs.transform.translation.x = pose->x;
    tfs.transform.translation.y = pose->y;
    tfs.transform.translation.z = 0.0;

    //将欧拉角转换成四元数
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,pose->theta);
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();

    //发布坐标
    broadcaster.sendTransform(tfs);
}

int main(int argc, char *argv[])
{
    //订阅乌龟的位姿
    ros::init(argc,argv,"turtle_pose_sub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose",100,doPose);

    ros::spin();
    return 0;
}

```

## 订阅方实现

```c++
#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    //创建监听者对象
    ros::init(argc,argv,"turtle_sub");
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Rate rate(1);

    while (ros::ok())
    {
        //定义任意点坐标在turtle1下的坐标
        geometry_msgs::PointStamped point_turtle;
        point_turtle.header.frame_id = "turtle1";
        point_turtle.header.stamp = ros::Time();
        point_turtle.point.x = 1; 
        point_turtle.point.y = 1;
        point_turtle.point.z = 0;
        try
        {
            //将turtle坐标下的点的坐标,转换成world坐标下的坐标
            geometry_msgs::PointStamped point_world;
            point_world = buffer.transform(point_turtle,"world");
            ROS_INFO("坐标点相对于 world 的坐标为:(%.2f,%.2f,%.2f)",
                     point_world.point.x,
                     point_world.point.y,
                     point_world.point.z);
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("发生异常.....");
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
```

## CMakeLists.txt

```cmake
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(turtlesim_tf_pub src/turtlesim_tf_pub.cpp)
add_executable(turtlesim_tf_sub src/turtlesim_tf_sub.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(turtlesim_tf_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(turtlesim_tf_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(turtlesim_tf_pub
  ${catkin_LIBRARIES}
)
target_link_libraries(turtlesim_tf_sub
  ${catkin_LIBRA
```

# 执行

启动rocore

```bash
roscore
```

启动turtlesim gui节点,键盘控制节点

```bash
rosrun turtlesim turtlesim_node 
```

```bash
rosrun turtlesim turtle_teleop_key 
```

执行

```bash
alvin@ros:~/catkin_ws$ rosrun tf_pratice turtlesim_tf_pub 
```

```bash
alvin@ros:~/catkin_ws$ rosrun tf_pratice turtlesim_tf_sub 
[ INFO] [1635691363.394649902]: 坐标点相对于 world 的坐标为:(6.54,6.54,0.00)
[ INFO] [1635691364.394588314]: 坐标点相对于 world 的坐标为:(6.54,6.54,0.00)
[ INFO] [1635691365.394655574]: 坐标点相对于 world 的坐标为:(6.54,6.54,0.00)
[ INFO] [1635691366.394559019]: 坐标点相对于 world 的坐标为:(6.54,6.54,0.00)
[ INFO] [1635691367.394593083]: 坐标点相对于 world 的坐标为:(6.54,6.54,0.00)
[ INFO] [1635691368.394657894]: 坐标点相对于 world 的坐标为:(6.54,6.54,0.00)
[ INFO] [1635691369.394621326]: 坐标点相对于 world 的坐标为:(6.54,6.54,0.00)
[ INFO] [1635691370.394648743]: 坐标点相对于 world 的坐标为:(6.54,6.54,0.00)
[ INFO] [1635691371.394659086]: 坐标点相对于 world 的坐标为:(6.54,6.54,0.00)
[ INFO] [1635691372.394657664]: 坐标点相对于 world 的坐标为:(6.54,6.54,0.00)
```

