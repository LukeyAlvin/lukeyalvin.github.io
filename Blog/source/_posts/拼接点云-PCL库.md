---
title: ch5_拼接点云(PCL库)
tags: [SLAM实践]
date: {{ date }}
comments: true
mathjax: true
categories: SLAM十四讲

---

{%cq%}

程序提供了五张 RGB-D 图像，并且知道了每个图像的内参和外参。根据 RGB-D 图像和相机内参，我们可以计算任何一个像素在相机坐标系下的位置。同时，根据相机位姿，又能计算这些像素在世界坐标系下的位置。如果把所有像素的空间坐标都求出来，相当于构建一张类似于地图的东西。

{%endcq%}

<!-- more -->

pose.txt 文件给出了五张图像的相机位姿（以 $T_{wc} $形式）。位姿记录的形式是平移向量加旋转四元数：
$[x, y, z, q_x , q_y , q_z , q_w ]$,其中 $q_w $是四元数的实部。

```txt
-0.228993 0.00645704 0.0287837 -0.0004327 -0.113131 -0.0326832 0.993042
-0.50237 -0.0661803 0.322012 -0.00152174 -0.32441 -0.0783827 0.942662
-0.970912 -0.185889 0.872353 -0.00662576 -0.278681 -0.0736078 0.957536
-1.41952 -0.279885 1.43657 -0.00926933 -0.222761 -0.0567118 0.973178
-1.55819 -0.301094 1.6215 -0.02707 -0.250946 -0.0412848 0.966741
```

案例主要完成两件事：

(1). 根据内参计算一对 RGB-D 图像对应的点云；
(2). 根据各张图的相机位姿（也就是外参），把点云加起来，组成地图。

````cpp
#include <iostream>
#include <fstream>
using namespace std;
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char **argv)
{
    vector<cv::Mat> colorImgs, depthImgs; // 彩色图和深度图
    // 定义相机位姿容器  用来存储相机位姿
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
    // 读入五张图像的相机位姿（以 Twc 形式）
    ifstream fin("./pose.txt");
    if (!fin)
    {
        cerr << "请在有pose.txt的目录下运行此程序" << endl;
        return 1;
    }
    //读入五张图片
    for (int i = 0; i < 5; i++)
    {
        boost::format fmt("./%s/%d.%s"); //图像文件格式
        // 读入彩色图放进容器中
        colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
        // 读入深度图放进容器中
        depthImgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "pgm").str(), -1)); // 使用-1读取原始图像

        // 读入pose中的位姿记录（位姿记录的形式是平移向量加旋转四元数）
        double data[7] = {0};
        for (auto &d : data)
            fin >> d;
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]); // qw qx qy qz
        Eigen::Isometry3d T(q);                                   // 由四元数构造变换矩阵T
        // 平移
        T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2])); // x y z pretanslate 相当于左乘
        // 将位姿信息传入位姿容器
        poses.push_back(T);
    }

    // 计算点云并拼接
    // 相机内参
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;
    cout << "正在将图像转换为点云..." << endl;

    // 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    // 新建一个点云
    PointCloud::Ptr pointCloud(new PointCloud);
    // 循环每一张图片
    for (int i = 0; i < 5; i++)
    {
        cout << "第" << i + 1 << "张图像转换中" << endl;
        cv::Mat color = colorImgs[i]; // 读取彩色图
        cv::Mat depth = depthImgs[i]; // 读取对应的深度图
        Eigen::Isometry3d T = poses[i]; // 读入相机位姿
        // 将像素坐标转根据相机位姿换成世界坐标
        for (int v = 0; v < color.rows; v++)
        {
            for (int u = 0; u < color.cols; u++)
            {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
                if (d == 0)
                    continue; // 为0表示没有测量到
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;   // Z
                point[0] = (u - cx) * point[2] / fx; // X
                point[1] = (v - cy) * point[2] / fy; // Y
                Eigen::Vector3d pointWorld = T * point; // 得到世界坐标系下的点云

                PointT p; // XYZRGB类型的p
                // 前三位存储点云的世界坐标
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2]; 
                // 后三位存储点云的bgr值
                p.b = color.data[v * color.step + u * color.channels()];
                p.g = color.data[v * color.step + u * color.channels() + 1];
                p.r = color.data[v * color.step + u * color.channels() + 2]; 
                pointCloud->points.push_back(p);
            }
        }
    }
    pointCloud->is_dense = false;
    cout << "点云一共有" << pointCloud->size() << "个点。" << endl;
    pcl::io::savePCDFileBinary("myPCL.pcd",*pointCloud);
    return 0;
}

````

CMakeLists.txt

```cmake
cmake_minimum_required( VERSION 2.8 )
project( joinMap )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++14" )
set(CMAKE_BUILD_TYPE Debug)

# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen 
include_directories( "/usr/include/eigen3/" )

# pcl 
find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

add_executable( joinMap joinMap.cpp )
target_link_libraries( joinMap ${OpenCV_LIBS} ${PCL_LIBRARIES} )
```

![PCL](PCL.PNG)

