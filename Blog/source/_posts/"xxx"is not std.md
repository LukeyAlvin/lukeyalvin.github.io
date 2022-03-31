---
title: fatal  error ‘xxxx’ was not declared in this scope
tags: [BUG,slam]
date: {{ date }}
comments: true
mathjax: true
categories: slam
---

{%cq%}
   运行slam14_ch3代码visualizeGeometry报错error: ‘xxxx’ was not declared in this scope

{%endcq%}

<!-- more -->

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-13_21-40.png"/>

亲测方案：CMakeLists.txt 添加 set(CMAKE_CXX_STANDARD 14)
或者：set(CMAKE_CXX_FLAGS "-std=c++14 -O2 ${SSE_FLAGS} -msse4") 

```cmake
cmake_minimum_required( VERSION 2.8 )
project( visualizeGeometry )

set(CMAKE_CXX_STANDARD 14)

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

# 添加Pangolin依赖
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable( visualizeGeometry visualizeGeometry.cpp )
target_link_libraries( visualizeGeometry ${Pangolin_LIBRARIES} )

```

