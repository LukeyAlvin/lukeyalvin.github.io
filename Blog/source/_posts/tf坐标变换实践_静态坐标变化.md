---
title: tf坐标变换实践_静态坐标变化
tags: [ros,tf坐标变换]
data: {{ date }}
comments: true
mathjax: true
categories: ROS
---

{%cq%}
    案例描述:现有一机器人模型，核心构成包含主体与雷达，各对应一坐标系，坐标系的原点分别位于主体与雷达的物理中心，已知雷达原点相对于主体原点位移关系如下: x=0.2 y=0.0 z=0.5。当前雷达检测到一障碍物，在雷达坐标系中障碍物的坐标为 (2.0 3.0 5.0),请问，该障碍物相对于主体的坐标是多少？
{%endcq%}
<!-- more -->

**实现分析:**

1. 坐标系相对关系，可以通过发布方发布
2. 订阅方，订阅到发布的坐标系相对关系，再传入坐标点信息(可以写死)，然后借助于 tf 实现坐标变换，并将结果输出

**实现流程:**

1. 新建功能包，添加依赖
2. 编写发布方实现
3. 编写订阅方实现
4. 执行并查看结果

# 命令实现

## 发布者命令实现

```bash
rosrun tf2_ros static_transform_publisher 0.2 0 0.5 0 0 0 /baselink /laser
```

解释:rosrun tf2_ros static_transform_publisher x偏移量 y偏移量 z偏移量 z偏航角度 y俯仰角度 x翻滚角度 父级坐标系 子级坐标系

## 查看发布信息

```bash
alvin@ros:~/catkin_ws$ rostopic list
/rosout
/rosout_agg
/tf_static
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
alvin@ros:~/catkin_ws$ rostopic echo /tf_static
transforms: 
  - 
    header: 
      seq: 1
      stamp: 
        secs: 1635601530
        nsecs: 483203886
      frame_id: "base_link"
    child_frame_id: "laser"
    transform: 
      translation: 
        x: 0.2
        y: 0.0
        z: 0.5
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
---
```

# C++代码实现

## 发布者实现

```c++
#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

 int main(int argc, char *argv[])
 {
    ros::init(argc,argv,"static_pub");
    //创建发布者对象
    tf2_ros::StaticTransformBroadcaster broadcaster;
    
    //坐标信息
    geometry_msgs::TransformStamped tfs;
    tfs.header.seq = 1;
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = "base_link";
    tfs.child_frame_id = "laser";
    tfs.transform.translation.x = 0.2;
    tfs.transform.translation.y = 0;
    tfs.transform.translation.z = 0.5;

    //将欧拉角转换成四元数
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,0);
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();
    
    //发布坐标
    broadcaster.sendTransform(tfs);

    ros::spin()
    return 0;
 }
```

## 订阅者实现

```c++
#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"static_sub");

    //创建监听者对象
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Rate rate(1);
    while(ros::ok()){
        //定义某坐标点在laser坐标系下的值
        geometry_msgs::PointStamped point_laser;
        point_laser.header.frame_id = "laser";
        point_laser.header.stamp = ros::Time::now();
        point_laser.point.x = 1 ;
        point_laser.point.y = 2 ;
        point_laser.point.z = 7.3;

        try
        {
            //将laser坐标系的点的坐标,转换成基坐标系下的坐标
            geometry_msgs::PointStamped point_base;
            point_base = buffer.transform(point_laser,"base_link");
            ROS_INFO("转换后的数据:(%.2f,%.2f,%.2f),参考的坐标系是:%s",
                     point_base.point.x,
                     point_base.point.y,
                     point_base.point.z,
                     point_base.header.frame_id.c_str());
        }
        catch(const std::exception& e)
        {
             ROS_INFO("程序异常.....");
        }
        
        
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
```

## CMakeLists.txt

```cmake
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(static_tf_pub src/static_tf_pub.cpp)
add_executable(static_tf_sub src/static_tf_sub.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(static_tf_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(static_tf_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(static_tf_pub
  ${catkin_LIBRARIES}
)
target_link_libraries(static_tf_sub
  ${catkin_LIBRARIES}
)
```

# 执行结果

```bash
alvin@ros:~/catkin_ws$ rosrun tf_pratice static_tf_pub 

```

```bash
alvin@ros:~/catkin_ws$ rosrun tf_pratice static_tf_sub 
[ INFO] [1635601858.503067357]: 程序异常.....
[ INFO] [1635601859.503191179]: 转换后的数据:(1.20,2.00,7.80),参考的坐标系是:base_link
[ INFO] [1635601860.503162320]: 转换后的数据:(1.20,2.00,7.80),参考的坐标系是:base_link
[ INFO] [1635601861.503066538]: 转换后的数据:(1.20,2.00,7.80),参考的坐标系是:base_link
[ INFO] [1635601862.503088002]: 转换后的数据:(1.20,2.00,7.80),参考的坐标系是:base_link
[ INFO] [1635601863.503098757]: 转换后的数据:(1.20,2.00,7.80),参考的坐标系是:base_link
[ INFO] [1635601864.503162287]: 转换后的数据:(1.20,2.00,7.80),参考的坐标系是:base_link
```

