---
title: fatal error Eigen/Core No such file or directory
tags: [BUG,eigen]
date: {{ date }}
comments: true
mathjax: true
categories: c++

---

{%cq%}
   已添加eigen库的路径到IncludePath，include<Eigen/Dense>也没有小灯泡，可是编译出错：fatal error: Eigen/Dense: No such file or directory 

{%endcq%}

<!-- more -->

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-13_10-31.png"/>

# 方案一:修改vscode配置文件

1. 添加eigen库的路径到IncludePath. （c_cpp_properties.json）

   > "/usr/include/eigen3/**",
   > "/usr/include/eigen3/"         

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-13_10-31_2.png" width="50%"/>

vscode中c_cpp_properties.json的"includePath"告诉vscode插件在哪里找到头文件，便于进行源码查看和debug，并没有告诉gcc编译器这个路径。

2. 添加eigen库的路径到编译参数"args".（tasks.json）

在"args"里加入如下代码,task："args"负责gcc等编译器的编译指令。

> "-I",
>
> "/usr/include/eigen3"

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-13_10-31_1.png" width="50%"/>

# 方案二:使用命令

命令`g++ eigenMatrix.cpp  -I /usr/include/eigen3 -o main`的意思就是编译代码,指定头文件位置,并输出可执行文件

```bash
alvin@ros:~/slam_ws/ch3/useEigen$ g++ eigenMatrix.cpp  -I /usr/include/eigen3 -o main
alvin@ros:~/slam_ws/ch3/useEigen$ ./main 
1 2 3
4 5 6
```

# 方案三:使用CMake

用vscode编写这类的调用第三方，其实最好的方式是用cmake，

首先，配置c_cpp_properties.json中的includepath，这里配置的目的只是为了在vscode里没有红色波浪线，看起来美观。

这里和方案一的第一步是一样的!

项目结构如下：

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-13_10-38.png"/>

CMakeLists.txt内容如下：

```cmake
cmake_minimum_required (VERSION 2.8.11)
project (demo)
find_package(Eigen3 REQUIRED)
include_directories(/usr/include/eigen3)
add_executable (eigenMatrix eigenMatrix.cpp)
```

运行代码:

```bash
mkdir build && cd build
cmake ..
make
./eigenMatrix
```

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-13_10-39.png" width="50%"/>