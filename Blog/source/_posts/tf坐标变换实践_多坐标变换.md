---
title: tf坐标变换实践_多坐标变换
tags: [ros,tf坐标变换]
date: 2021-11-01 10:44:43
comments: true
mathjax: true
categories: ROS

---

{%cq%}
    案例描述:现有坐标系统，父级坐标系统 world,下有两子级系统 son1，son2，son1 相对于 world，以及 son2 相对于 world 的关系是已知的，求 son1原点在 son2中的坐标，又已知在 son1中一点的坐标，要求求出该点在 son2 中的坐标.已知son1和son2相对于world的偏移量分别是(0.2 0.8 0.3 0 0 0)和(0.5 0 0 0 0 0),求解点一直某一点在son1坐标系中的坐标为(1,2,3),求该点在son2中的坐标?

{%endcq%}

<!-- more -->

**案例分析:**

1. 首先，需要发布 son1 相对于 world，以及 son2 相对于 world 的坐标消息
2. 然后，需要订阅坐标发布消息，并取出订阅的消息，借助于 tf2 实现 son1 和 son2 的转换
3. 最后，还要实现坐标点的转换

# C++代码实现

## 发布方

**前提:** 发布两个子坐标系节点(也是静态坐标节点)

```xml
<launch>
    <node pkg="tf2_ros" name="son1" type="static_transform_publisher" args="0.2 0.8 0.3 0 0 0 /world /son1" output="screen" />
    <node pkg="tf2_ros" name="son2" type="static_transform_publisher" args="0.5 0 0 0 0 0 /world /son2" output="screen" />
</launch>
```

## 订阅方

```c++
#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"dynamic_pub");
    //创建监听对象,接受发布的静态坐标
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Rate rate(1);
    while (ros::ok())
    {
        try
        {
            //解析son1中的点相对于son2的坐标
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("son2","son1",ros::Time(0));
            ROS_INFO("Son1 相对于 Son2 的坐标关系:父坐标系ID=%s,子坐标系ID=%s,坐标关系:x=%.2f,y=%.2f,z=%.2f",
                    tfs.header.frame_id.c_str(),tfs.child_frame_id.c_str(),
                    tfs.transform.translation.x,
                    tfs.transform.translation.y,
                    tfs.transform.translation.z);

            //坐标点解析
            geometry_msgs::PointStamped point_son1;
            point_son1.header.frame_id = "son1";
            point_son1.header.stamp = ros::Time::now();
            point_son1.point.x = 1;
            point_son1.point.y = 2;
            point_son1.point.z = 3;

            geometry_msgs::PointStamped point_son2;
            //PS:调用 transform 必须包含头文件"tf2_geometry_msgs/tf2_geometry_msgs.h"
            point_son2 = buffer.transform(point_son1,"son2");
            ROS_INFO("在 Son2 中的坐标:x=%.2f,y=%.2f,z=%.2f",
                    point_son2.point.x,
                    point_son2.point.y,
                    point_son2.point.z);
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

# CMakeLists.txt

```cmake
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(dynamic_tf src/dynamic_tf.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(dynamic_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(dynamic_tf
  ${catkin_LIBRARIES}
)
```

