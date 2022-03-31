---
title: 了解cmake编译_hello world
tags: [SLAM实践]
date: {{ date }}
comments: true
mathjax: true
categories: SLAM十四讲
---

{%cq%}

理论上说，任意一个 C++ 程序都可以用 g++ 来编译。但当程序规模越来越大时，一个工程可能有许多个文件夹和里边的源文件，这时输入的编译命令将越来越长。通常一个小型 c++ 项目含有十几个类，各类间还存在着复杂的依赖关系。其中一部分要编译成可执行文件，另一部分编译成库文件。如果仅靠 g++ 命令，我们需要输入大量的编译指令，整个编译过程会变得异常繁琐。因此，对于 C++ 项目，使用一些工程管理工具会更加高效。

{%endcq%}

<!-- more -->

库文件：用来存放函数或者类中函数的具体实现

在一个 C++ 工程中，并不是所有代码都会编译成可执行文件。只有带有 main 函数的文件才会生成可执行程序。而另一些代码，我们只想把它们打包成一个东西，供其他程序调用。这个东西叫做库。

```c++
//这是一个库文件
#include <iostream>
using namespace std;

void printHello()
{
    cout<<"Hello SLAM"<<endl;
}
```

对于库的使用者，只要拿到了头文件和库文件，就可以调用这个库了。

头文件：存放方法或者类相关的声明

```c++
#ifndef LIBHELLOSLAM_H_
#define LIBHELLOSLAM_H_
// 上面的宏定义是为了防止重复引用这个头文件而引起的重定义错误

void printHello();

#endif
```

主函数：main函数

```c++
#include "libHelloSLAM.h"

// 使用 libHelloSLAM.h 中的 printHello() 函数
int main( int argc, char** argv )
{
    printHello();
    return 0;
}
```

当然，我们需要使用CMakeLists.txt文件帮助我们进行编译

```cmake
# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( HelloSLAM )

# 设置编译模式
set( CMAKE_BUILD_TYPE "Debug" )

# 添加一个库
add_library( hello libHelloSLAM.cpp )
# 共享库
add_library( hello_shared SHARED libHelloSLAM.cpp )
# 将库文件链接到可执行程序上
target_link_libraries( useHello hello_shared )

add_executable( useHello useHello.cpp )
```

运行

```bash
mkdir build
cd build 
cmake ..
make
```

