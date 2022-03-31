---
title: 使用OpenCV操作图像
tags: [SLAM实践]
date: {{ date }}
comments: true
mathjax: true
categories: SLAM十四讲
---

{%cq%}

OpenCV 提供了大量的开源图像算法，是计算机视觉中使用极广的图像处理算法库。在ubuntu 下，你可以选择从源代码安装和只安装库文件两种方式。

{%endcq%}

<!-- more -->

源代码安装：

从 http://opencv.org/downloads.html 中下载，选择 OpenCV for Linux 版本即可

在编译之前，先来安装 OpenCV 的依赖项

```bash
sudo apt-get install build-essential libgtk2.0-dev libvtk5-dev libjpeg-dev libtiff4-dev libjasper-dev libopenexr-dev libtbb-dev
```

编译：

```bash
mkdir build
cd build 
cnake ..
make -j4
sudo make install
```

案例：

```cpp
#include <iostream>
#include <chrono>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
    // 读取argv[1]指定的图像
    cv::Mat image;
    image = cv::imread(argv[1]); // cv::imread函数读取指定路径下的图像

    //数据不存在,可能是文件不存在
    if (image.data == nullptr)
    {
        // cerr：输出到标准错误的ostream对象，常用于程序错误信息；
        cerr << "文件" << argv[1] << "不存在" << endl;
        return 0;
    }

    // 文件顺利读取, 首先输出一些基本信息
    cout << "图像的宽为：" << image.cols
         << ",高为：" << image.rows
         << "通道数为：" << image.channels() << endl;
    cv::imshow("image", image); // 用cv::imshow显示图像
    cv::waitKey(0);             // 暂停程序,等待一个按键输入

    // 判断image的类型
    if (image.type() != CV_8UC1 && image.type() != CV_8UC3) // 图像既不是单通道也不是三通道
    {
        // 图像类型不符合要求
        cout << "请输入一张彩色图或者灰度图" << endl;
        return 0;
    }

    // 遍历图像, 请注意以下遍历方式亦可使用于随机像素访问
    // 使用 std::chrono 来给算法计时
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (size_t y = 0; y < image.rows; y++)
    {
        // 用cv::Mat::ptr获得图像的行指针
        unsigned char *row_ptr = image.ptr<unsigned char>(y); // row_ptr是第y行的头指针
        for (size_t x = 0; x < image.cols; x++)
        {
            // 访问位于 x,y 处的像素
            unsigned char *data_ptr = &row_ptr[x * image.channels()]; // data_ptr 指向待访问的像素数据
            // 输出该像素的每个通道,如果是灰度图就只有一个通道
            for (int i = 0; i != image.channels(); i++)
            {
                unsigned char data = data_ptr[i]; // data为I(x,y)第c个通道的值
            }
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "遍历图像用时：" << time_used.count() << " 秒。" << endl;
    
    // 关于 cv::Mat 的拷贝
    // 直接赋值并不会拷贝数据,使用clone函数来拷贝数据
    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0,0,100,100) ).setTo(0); // 将左上角100*100的块置零
    cv::imshow("image_clone",image_clone);
    cv::waitKey(0);

    cv::destroyAllWindows();

    return 0;
}

```

CMakeLsits.txt

```cmake
cmake_minimum_required( VERSION 2.8 )
project( imageBasics )

# 添加c++ 14标准支持
set( CMAKE_CXX_FLAGS "-std=c++14" )

# 寻找OpenCV库
find_package( OpenCV 3 REQUIRED )
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( imageBasics imageBasics.cpp )
# 链接OpenCV库
target_link_libraries( imageBasics ${OpenCV_LIBS} )
```

