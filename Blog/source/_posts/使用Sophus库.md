---
title: 使用Sophus库
tags: [SLAM实践]
date: {{ date }}
comments: true
mathjax: true
categories: SLAM十四讲
---

{%cq%}

Sophus是直接在 Eigen 基础上开发的，我们不需要要安装额外的依赖库。由于历史原因，Sophus 早期版本只提供了双精度的李群/李代数类。后续版本改写成了模板类。模板类的 Sophus 中可以使用不同精度的李群/李代数，但同时增加了使用难度。这里使用的是非模板类。

{%endcq%}

<!-- more -->

```cpp
#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
// 这里使用的是绝对路径，所以在CMakeLsits.txt里并没有包含Sophus库
#include "/home/alvin/workspace/slam_ws/slambook/3rdparty/Sophus/sophus/so3.cpp"
#include "/home/alvin/workspace/slam_ws/slambook/3rdparty/Sophus/sophus/se3.cpp"

int main(int argc, char **argv)
{
    // 沿Z轴转45s度的旋转矩阵
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d(0, 0, 1)).matrix();
    cout.precision(3); //保留三位小数
    cout << "R = " << endl
         << R << endl;

    // 构造李群
    Sophus::SO3 SO3_R(R);              // Sophus::SO(3)可以直接从旋转矩阵构造
    Sophus::SO3 SO3_v(0, 0, M_PI / 4); // 亦可从旋转向量构造
    Eigen::Quaterniond q(R);           // 或者四元数
    Sophus::SO3 SO3_q(q);
    // 上述表达方式都是等价的
    // 输出SO(3)时，以so(3)形式输出
    cout << "从旋转矩阵构造SO(3): " << SO3_R << endl;
    cout << "从旋转向量构造SO(3): " << SO3_v << endl;
    cout << "从四元数构造SO(3):" << SO3_q << endl;

    // 使用对数映射获得它的李代数
    Eigen::Vector3d so3 = SO3_R.log();
    cout << "SO3的李代数so3：" << so3.transpose() << endl; // transpose纯粹是为了输出美观一些
    // hat 为向量到反对称矩阵
    cout << "so3的反对称矩阵：\n"
         << Sophus::SO3::hat(so3) << endl;
    // 相对的，vee为反对称到向量
    cout << "反对称矩阵到其向量：\n"
         << Sophus::SO3::vee(Sophus::SO3::hat(so3)).transpose() << endl;
    ; // transpose纯粹是为了输出美观一些

    // 增量扰动模型的更新
    Eigen::Vector3d update_so3(1e-4, 0, 0); //假设更新量为这么多
    Sophus::SO3 SO3_update = Sophus::SO3::exp(update_so3) * SO3_R;
    cout << "SO3 updated = \n"
         << SO3_update << endl;

    cout << "************SE*************" << endl;
    Eigen::Matrix3d R1 = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d(1, 0, 0)).matrix();
    Eigen::Vector3d t(1, 0, 0); // 沿X轴平移1
    Sophus::SE3 SE3_Rt(R1, t);  // 从R,t构造SE(3)
    Sophus::SE3 SE3_qt(q, t);   // 从q,t构造SE(3)
    cout << "从R,t构造的SE：\n"
         << SE3_Rt << endl;
    cout << "从q,t构造的SE：\n"
         << SE3_qt << endl;

    // 李代数se(3) 是一个六维向量，方便起见先typedef一下
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    cout << "se3:\n"
         << se3.transpose() << endl; // 观察输出，会发现在Sophus中，se(3)的平移在前，旋转在后.
    
    // 同样的，有hat和vee两个算符
    cout << "se3的反对称矩阵：\n" << Sophus::SE3::hat(se3) << endl;
    cout << "se3反对称矩阵到其向量：\n" << Sophus::SE3::vee( Sophus::SE3::hat(se3) ).transpose() << endl;


    // 最后，演示一下更新
    Vector6d update_se3;//更新量
    update_se3.setZero();
    update_se3(0,0) = 1e-4d;
    Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3)*SE3_Rt;
    cout << "SE3 updated = \n" << SE3_updated.matrix() << endl; 
    return 0;
}
```

## CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "-std=c++14")

project(useSophus)
include_directories("/usr/include/eigen3")

add_executable(useSophus useSophus.cpp)
```

