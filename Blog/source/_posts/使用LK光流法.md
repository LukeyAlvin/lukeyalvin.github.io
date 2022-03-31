---
title: ch8_使用LK光流法
tags: [SLAM实践]
date: {{ date }}
comments: true
mathjax: true
categories: SLAM十四讲

---

{%cq%}

光流是一种描述像素随着时间，在图像之间运动的方法。随着时间的经过，同一个像素会在图像中运动，而我们希望追踪它的运动过程。计算部分像素运动的称为稀疏光流，计算所有像素的称为稠密光流。稀疏光流以$ Lucas-Kanade$ 光流为代表，并可以在 SLAM 中用于跟踪特征点位置。

{%endcq%}

<!-- more -->

![image-20220329104428515](image-20220329104428515.png)

# LK光流法原理

在$ LK $光流中，我们认为来自相机的图像是随时间变化的。图像可以看作时间的函数：$I(t)$。那么，一个在$ t $时刻，位于$ (x, y)$ 处的像素，它的灰度可以写成
$$
I(x, y, t).
$$
灰度不变假设：
$$
I(x+d_x,y+d_y,t+d_t)=I(x, y, t)
$$
对左边进行泰勒展开，保留一阶项，得：
$$
I (x + dx, y + dy, t + dt) \approx I (x, y, t)
+\frac{\partial I}{\partial x}d_x
+\frac{\partial I}{\partial y}d_y
+\frac{\partial I}{\partial t}d_t
$$
假设了灰度不变:
$$
\frac{\partial I}{\partial x}d_x
+\frac{\partial I}{\partial y}d_y
+\frac{\partial I}{\partial t}d_t=0
$$
两边除以 $dt$:
$$
\frac{\partial I}{\partial x}\frac{d_x}{d_t}
+\frac{\partial I}{\partial y}\frac{d_y}{d_t}
=-\frac{\partial I}{\partial t}
$$
其中: $dx/dt $为像素在 $x $轴上运动速度，记为$u$， $dy/dt $为像素在 $y$轴上运动速度，记为$v$

$\partial I/\partial x$为图像在该点处 $x$ 方向的梯度，记为$I_x$，$\partial I/\partial y$为图像在该点处 $y$ 方向的梯度，记为$I_y$

图像灰度对时间的变化量记为 $I_t$
$$
\begin{bmatrix}
I_x & I_y
\end{bmatrix}
\begin{bmatrix}
u \\ v
\end{bmatrix}
= - I_t
$$
我们想计算的是像素的运动$ u, v$，但是该式是带有两个变量的一次方程，仅凭它无法计算出$ u, v$。因此，必须引入额外的约束来计算$ u, v$。在$ LK $光流中，我们假设某一个窗口内的像素具有相同的运动。

考虑一个大小为 $w × w$ 大小的窗口，它含有 $w^2 $数量的像素。由于该窗口内像素具有同样的运动，因此我们共有 $w^2 $个方程：
$$
\begin{bmatrix}
I_x & I_y
\end{bmatrix}_k
\begin{bmatrix}
u \\ v
\end{bmatrix}
= - I_{tk},k=1,....,w^2
$$
即为：
$$
A = 
\begin{bmatrix}
\begin{bmatrix}
I_x & I_y
\end{bmatrix}_1\\.\\.\\
\begin{bmatrix}
I_x & I_y
\end{bmatrix}_k
\end{bmatrix},
b=\begin{bmatrix}
I_{t1} \\.\\.\\I_{tk} 
\end{bmatrix}
$$
于是整个方程为：
$$
A\begin{bmatrix}
u\\u
\end{bmatrix}=-b
$$
这是一个关于 u, v 的超定线性方程，传统解法是求最小二乘解。最小二乘在很多时候都用到过：
$$
\begin{bmatrix}
u\\u
\end{bmatrix}^*
=-(A^TA)^{-1}A^Tb
$$
这样就得到了像素在图像间的运动速度 $u, v$

# 使用LK光流法

## 光流追踪的原理

- cv2.FastFeatureDetector()：Fast角点检测器确定要追踪的特征点

- cv2.calcOpticalFlowPyrLK()： 追踪视频中的稀疏特征点

- cv2.calcOpticalFlowFarneback()： 追踪视频中的密集特征点

取第一帧，检测其中的一些 Fast角点，使用 Lucas-Kanade 光流迭代跟踪这些点。对于函数 cv2.calcOpticalFlowPyrLK() 传递前一帧、前一个点和下一帧。它返回下一个点以及一些状态编号，如果找到下一个点，则值为 1，否则为零。然后在下一步中迭代地将这些下一个点作为前一个点传递。

## 光流的两种方法

OpenCV提供了俩种算法计算光流，分别通过：cv2.calcOpticalFlowPyrLK()、cv2.calcOpticalFlowFarneback实现；

- 稀疏光流： 通过 Lucas-Kanade 方法计算稀疏特征集的光流（使用 角点检测算法检测到的角点）。

参数：

```cpp
void cv::calcOpticalFlowPyrLK	(	
        InputArray 	prevImg, //上一帧单通道灰度图
        InputArray 	nextImg, //下一帧单通道灰度图
        InputArray 	prevPts, //像素点上一帧二维坐标pts
        InputOutputArray 	nextPts,//像素点下一帧二维坐标pts
        OutputArray 	status,// 输出状态向量,（无符号字符）;如果找到相应特征的流，则向量的每个元素设置为1，否则设置为0。
        OutputArray 	err, //输出错误的矢量;如果未找到流，则未定义错误（使用status参数查找此类情况）。
        Size 	winSize = Size(21, 21),//每个金字塔等级的搜索窗口的winSize大小。
        int 	maxLevel = 3,// 基于0的最大金字塔等级数;如果设置为0，则不使用金字塔（单级），如果设置为1，则使用两个级别，依此类推;
        TermCriteria 	criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),//参数，指定迭代搜索算法的终止条件
        int 	flags = 0,
        double 	minEigThreshold = 1e-4 //算法计算光流方程的2x2正常矩阵的最小特征值，除以窗口中的像素数;
)	
```

- 密集光流： 通过 Gunner Farneback 来寻找密集光流。它计算帧中所有点的光流。

## 代码

在使用数据之前，需要根据采集时间，对数据进行一次时间上的对齐，以便对彩色图和深度图进行配对。TUM 提供了一个 python 脚本“asso-ciate.py”帮我们完成这件事。请把此文件放到数据集目录下，运行：

```bash
python associate.py rgb.txt depth.txt > associate.txt
```

主函数

```cpp
int main(int argc, char **argv)
{
    if (argc != 2)
    {
        cout << "输入数据集所在的路径！" << endl;
        return 1;
    }
    string path_to_dataset = argv[1];
    string associate_file = path_to_dataset + "/associate.txt";
    ifstream fin(associate_file);
    if (!fin)
    {
        cout << "associate.txt文件不存在！" << endl;
        return 1;
    }

    string rgb_file, depth_file, time_rgb, time_depth;
    list<cv::Point2f> keypoints; // 因为要删除跟踪失败的点，使用list
    cv::Mat color, depth, last_color;
    for (int index = 0; index < 100; index++)
    {
        fin >> time_rgb >> rgb_file >> time_depth >> depth_file;
        color = cv::imread(path_to_dataset + "/" + rgb_file);       // 读入彩色图
        depth = cv::imread(path_to_dataset + "/" + depth_file, -1); // 读入深度图
        // 1.对第一帧提取FAST特征点
        if (index == 0)
        {
            vector<cv::KeyPoint> kps;
            cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            detector->detect(color, kps); // 提取FAST特征点
            for (auto kp : kps)           // 把提取的特征点存入list
                keypoints.push_back(kp.pt);
            last_color = color;
            continue;
        }
        // 2.对其他帧使用LK跟踪特征点
        if (color.data == nullptr && depth.data == nullptr)
            continue;
        vector<cv::Point2f> next_keypoints;
        vector<cv::Point2f> prev_keypoints;
        for (auto kp : keypoints)
            prev_keypoints.push_back(kp); // 讲list中的数据赋给prev_keypoints
        vector<unsigned char> status;
        vector<float> error;

        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

        cv::calcOpticalFlowPyrLK(last_color, color, prev_keypoints, next_keypoints, status, error);

        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "LK Flow use time：" << time_used.count() << " seconds." << endl;

        // 把跟丢的点删掉
        int i = 0;
        for (auto iter = keypoints.begin(); iter != keypoints.end(); i++)
        {
            if (status[i] == 0)
            {
                iter = keypoints.erase(iter);
                continue;
            }
            *iter = next_keypoints[i];
            iter++;
        }
        cout << "跟踪到的点个数为： " << keypoints.size() << endl;
        if (keypoints.size() == 0)
        {
            cout << "all keypoints are lost." << endl;
            break;
        }

        // 画出 keypoints
        cv::Mat img_show = color.clone();
        for (auto kp : keypoints)
        {
            cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);
        }
        cv::imshow("corners", img_show);
        cv::waitKey(0);
        last_color = color;
    }
    return 0;
}
```

> 因为要删除跟踪失败的点，使用list,原因如下：
>
> 由于链表的存储方式并不是连续的内存空间，因此链表list中的迭代器只支持前移和后移，属于双向迭代器
>
> list的优点：
>
> > 采用动态存储分配，不会造成内存浪费和溢出
> > 链表执行插入和删除操作十分方便，修改指针即可，不需要移动大量元素

打印：

```C#
LK Flow use time：0.0157339 seconds.
跟踪到的点个数为： 1749
LK Flow use time：0.0144717 seconds.
跟踪到的点个数为： 1742
LK Flow use time：0.0186928 seconds.
跟踪到的点个数为： 1703
LK Flow use time：0.0153624 seconds.
跟踪到的点个数为： 1676
LK Flow use time：0.019426 seconds.
跟踪到的点个数为： 1664
LK Flow use time：0.0140176 seconds.
跟踪到的点个数为： 1656
LK Flow use time：0.0187153 seconds.
跟踪到的点个数为： 1641
LK Flow use time：0.0187284 seconds.
跟踪到的点个数为： 1634
```

CMakeLists.txt

```cmake
cmake_minimum_required( VERSION 2.8 )
project( useLK )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( useLK useLK.cpp )
target_link_libraries( useLK ${OpenCV_LIBS} )

```

![image-20220329144916230](image-20220329144916230.png)

LK 光流跟踪能够直接得到特征点的对应关系。这个对应关系就像是描述子的匹配，但实际上我们大多数时候只会碰到特征点跟丢的情况，而不太会遇到误匹配，这应该是光流相对于描述子的一点优势。但是，匹配描述子的方法在相机运动较大时仍能成功，而光流必须要求相机运动是微小的。从这方面来说，光流的鲁棒性比描述子差一些。

光流法参考：

- [OpenCV中的光流及视频特征点追踪](https://www.shouxicto.com/article/1871.html)
- [ calcOpticalFlowPyrLK()](https://docs.opencv.org/3.4.6/dc/d6b/group__video__track.html#ga473e4b886d0bcc6b65831eb88ed93323)
- [calcOpticalFlowFarneback()](https://docs.opencv.org/3.4.6/dc/d6b/group__video__track.html#ga5d10ebbd59fe09c5f650289ec0ece5af)

