---
title: ch7_求解PnP并使用BA优化
tags: [SLAM实践]
date: {{ date }}
comments: true
mathjax: true
categories: SLAM十四讲
---

{%cq%}

$PnP（Perspective-n-Point）$是求解 $3D $到$ 2D $点对运动的方法。它描述了当我们知道$n $个$ 3D$ 空间点以及它们的投影位置时，如何估计相机所在的位姿。如果两张图像中，其中一张特征点的 $3D$ 位置已知，那么最少只需三个点对（需要至少一个额外点验证结果）就可以估计相机运动。特征点的$ 3D $位置可以由三角化，或者由 $RGB-D$ 相机的深度图确定。因此，在双目或$ RGB-D$ 的视觉里程计中，我们可以直接使用$ PnP$ 估计相机运动。而在单目视觉里程计中，必须先进行初始化，然后才能使用 $PnP$。

{%endcq%}

<!-- more -->



# PnP中的BA问题

我们可以把 $PnP $问题构建成一个定义于李代数上的非线性最小二乘问题。前面说的线性方法，往往是**先求相**
**机位姿，再求空间点位置**，而非线性优化则是把它们都看成优化变量，放在一起优化。这是一种非常通用的求解方式，我们可以用它对$ PnP $或$ ICP $给出的结果进行优化。

## 1.重投影误差

在 $PnP $中，这个$ Bundle Adjustment $问题，是一个最小化重投影误差（Reprojection error）的问题。

![image-20220329214556296](image-20220329214556296.png)

该问题的误差项，是将像素坐标（观测到的投影位置）与 3D 点按照当前估计的位姿进行投影得到的位置相比较得到的误差，所以称之为重投影误差。如图，我们通过特征匹配，知道了 $p_1$和 $p_2$是同一个空间点$P$ 的投影，但是我们不知道相机的位姿。在初始值中，$P $的投影$\hat{p_2}$与实际的$ p_2$之间有一定的距离。于是我们调整相机的位姿，使得这个距离变小。不过，由于这个调整需要考虑很多个点，所以最后每个点的误差通常都不会精确为零。

考虑 $n$ 个三维空间点 $P $和它们的投影$ p$，我们希望计算相机的位姿 $R, t，$它的李代数表示为 $ξ$。假设某空间点坐标为$ P_i = [X_i , Y_i , Z_i ]^{T} $，其投影的像素坐标为 $u_i = [u_i , v_i ]^{T} $。
像素位置与空间点位置的关系如下：
$$
s_i
\left[
\begin{matrix}
u_i\\v_i\\1
\end{matrix}
\right]=
Kexp(ξ^{\land})
\left[
\begin{matrix}
X_i\\Y_i\\Z_i\\1
\end{matrix}
\right]
$$
写成矩阵形式：
$$
s_iu_i=Kexp(ξ^{\land})P_i
$$
由于相机位姿未知以及观测点的噪声，该等式存在一个误差，把误差求和，构建最小二乘问题，然后寻找最好的相机位姿，使它最小化:
$$
ξ^{*}=arg \underset{ξ}{min}\frac{1}{2}\sum^n_{i=1}\parallel u_i-\frac{1} {s_i}Kexp(ξ^{\land})P_i\parallel^2_2
$$
求每个误差项关于优化变量的导数，也就是线性化：
$$
e(x+\Delta x)\approx e(x)+J\Delta x
$$
$e $为像素坐标误差（2 维），$x$ 为相机位姿（6 维）时，$J$ 将是一个 2 × 6 的矩阵。

## 2.优化位姿

记变换到相机坐标系下的空间点坐标为 $P^′$:
$$
s
\left[
\begin{matrix}
u\\v\\1
\end{matrix}
\right]=
\left[
\begin{matrix}
f_x &0 &c_x\\0 &f_y &c_y\\0&0 &1
\end{matrix}
\right]
\left[
\begin{matrix}
X^{′}\\Y^{′}\\Z^{′}
\end{matrix}
\right]
$$
利用第 3 行消去 $s$（实际上就是$ P^{′ }$的距离），得：
$$
u=f_x\frac{X^′}{Z^′}+c_x\\
v=f_y\frac{Y^′}{Z^′}+c_y
$$
对 $ξ^{\land} $左乘扰动量$ δξ$，然后考虑 $e$ 的变化关于扰动量的导数。利用链式法则:
$$
\frac{\partial e}{\partial \delta \xi}=
\underset{\delta \xi \rightarrow0}{lim}
\frac{e(\delta \xi⊕\xi)}{\delta \xi}=
\frac{\partial e}{\partial P^′}
\frac{\partial P^′}{\partial \delta \xi}
$$
其中：$\frac{\partial e}{\partial P^′}$
$$
\frac{\partial e}{\partial P^′}=-
\begin{bmatrix}
\frac{\partial u}{\partial X^′}&
\frac{\partial u}{\partial Y^′}&
\frac{\partial u}{\partial Z^′}			\\ 
\frac{\partial v}{\partial X^′}&
\frac{\partial v}{\partial Y^′}&
\frac{\partial v}{\partial Z^′}
\end{bmatrix}=-
\begin{bmatrix}
\frac{f_x}{Z^′}&0&-\frac{f_xX^′}{Z^{′2}}\\
0&\frac{f_y}{Z^′}&-\frac{f_yY^′}{Z^{′2}}
\end{bmatrix}
$$
$\frac{\partial P^′}{\partial \delta \xi}$为变换后的点关于李代数的导数：
$$
\begin{align}
\frac{\partial P^′}{\partial \delta \xi}&=
\frac{\partial(TP)}{\partial \delta \xi} \\ &=
\underset{\delta \xi \rightarrow0}{lim}
\frac{exp(\delta \xi ^{\land})exp(\xi ^{\land})p-exp(\xi ^{\land})p}{\delta \xi}\\&\approx
\underset{\delta \xi \rightarrow0}{lim}
\frac{(I+\delta \xi ^{\land})exp(\xi ^{\land})p-exp(\xi ^{\land})p}{\delta \xi}\\&=
\underset{\delta \xi \rightarrow0}{lim}
\frac{\delta \xi ^{\land}exp(\xi ^{\land})p}{\delta \xi}\\&=
\underset{\delta \xi \rightarrow0}{lim}
\frac{\begin{bmatrix}
\delta \phi^{\land}&\delta \rho \\
0^T&0
\end{bmatrix}
\begin{bmatrix}
Rp+t \\
1
\end{bmatrix}
}{\delta \xi}\\&=
\underset{\delta \xi \rightarrow0}{lim}
\frac{\begin{bmatrix}
\delta \phi^{\land}(Rp+t)+\delta \rho  \\
0
\end{bmatrix}
}
{\delta \xi}\\&=
\begin{bmatrix}
I &-(RP+t)^{\land}\\
0^{T}&0^{T}
\end{bmatrix}\\&=
\begin{bmatrix}
I &-P^{′\land}\\
0^{T}&0^{T}
\end{bmatrix}\\&=
(TP)^⊙
\end{align}
$$
两项相乘：
$$
\begin{align}
\frac{\partial e}{\partial \delta \xi}&=
\frac{\partial e}{\partial P^′}
\frac{\partial P^′}{\partial \delta \xi}\\&=-
\begin{bmatrix}
\frac{f_x}{Z^′}&0&-\frac{f_xX^′}{Z^{′2}}
&-\frac{f_xX^′Y^′}{Z^{′2}}&f_x+\frac{f_xX^2}{Z^{′2}}&-\frac{f_xY^′}{Z^{′}}
\\
0&\frac{f_y}{Z^′}&-\frac{f_yY^′}{Z^{′2}}
&-f_y-\frac{f_yY^{′2}}{Z^{′2}}&\frac{f_yX^′Y^′}{Z^{′}}&\frac{f_yX^′}{Z^{′}}
\end{bmatrix}\\&=
J
\end{align}
$$
这个雅可比矩阵描述了重投影误差关于相机位姿李代数的一阶变化关系，

## 3.优化空间点位置

讨论 $e$关于空间点 $P $的导数
$$
\frac{\partial e}{\partial P}=
\frac{\partial e}{\partial P^′}
\frac{\partial P^′}{\partial P}
$$
由于：
$$
\begin{align}
P^′&=exp(\xi ^{\land})P=RP+t\\
\frac{\partial P^′}{\partial P}&=R
\end{align}
$$
所以：
$$
\frac{\partial e}{\partial P}=-
\begin{bmatrix}
\frac{f_x}{Z^′}&0&-\frac{f_xX^′}{Z^{′2}}\\
0&\frac{f_y}{Z^′}&-\frac{f_yY^′}{Z^{′2}}
\end{bmatrix}R
$$
**我们推导了观测相机方程关于相机位姿与特征点的两个导数矩阵。它们十分重要，能够在优化过程中提供重要的梯度方向，指导优化的迭代。**

# 求解PnP

$PnP $问题有很多种求解方法，例如用三对点估计位姿的 $P3P$，直接线性变换$(DLT)$，$EPnP(Efficient PnP)$，$UPnP$等等。此外，还能用非线性优化的方式，构建最小二乘问题并迭代求解，也就是万金油式的$ Bundle Adjustment$。

其中$P3P$利用了三角形相似性质，求解投影点 $a, b, c$ 在相机坐标系下的 3D 坐标，最后把问题转换成一个 3D 到 3D 的位姿估计问题($ICP$问题)。但是，$P3P$ 只利用三个点的信息。当给定的配对点多于 3 组时，难以利用更多的信息；而且，如果 3D 点或 2D 点受噪声影响，或者存在误匹配，则算法失效。

在 $SLAM$ 当中，通常的做法是先使用 $P3P/EPnP$ 等方法估计相机位姿，然后构建最小二乘优化问题对估计值进行调整$（Bundle Adjustment）$。

## 1.特征点匹配

```cpp
// 1. 特征匹配
void find_feature_matches(
    const Mat &img_1,
    const Mat &img_2,
    std::vector<KeyPoint> &keypoints_1,
    std::vector<KeyPoint> &keypoints_2,
    std::vector<DMatch> &matches)
{
    // 初始化
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);
    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);
    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    matcher->match(descriptors_1, descriptors_2, match);
    //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        double dist = match[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }
    cout << "max_dist: " << max_dist << endl;
    cout << "min_dist: " << min_dist << endl;
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (match[i].distance <= max(2*min_dist,30.0))
        {
            matches.push_back(match[i]);
        }
    }
}
```

## 2.建立3D点

`queryIdx` 代表的特征点序列是 `keypoints1` 中的，`trainIdx `代表的特征点序列是` keypoints2 `中的，此时这两张图中的特征点相互匹配。

```cpp
for (DMatch m : matches)
{
        cout << "queryIdx: " << keypoints_1[m.queryIdx].pt << endl;
        cout << "trainIdx: " << keypoints_2[m.trainIdx].pt << endl;
}
```

这两行代码返回的类型都是`Point2d`，对同一个匹配点对，总有第一帧图像的像素点坐标`keypoints_1[m.queryIdx].pt`与第二帧图像的像素点坐标`keypoints_2[m.trainIdx].pt`相对应。因此，如果匹配点对为79,则存在79对相互对应的`keypoints_1[m.queryIdx].pt`与`keypoints_2[m.trainIdx].pt`

```c#
--------第77组匹配点----------
queryIdx: [207.825, 286.655]
trainIdx: [186.325, 290.238]
--------第78组匹配点----------
queryIdx: [383.4, 326.07]
trainIdx: [347.569, 336.819]
--------第79组匹配点----------
queryIdx: [351.152, 229.324]
trainIdx: [333.236, 240.073]
```

由于我们需要3D-2D，因此需要将一对匹配点的其中的一个转换成相机坐标系：

```cpp
// 像素坐标系转换成相机归一化平面的投影
Point2d pixel2cam(const Point2d &p, const Mat &K)
{
    return Point2d(
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}
for (DMatch m : matches)
{
    ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)]; // 获取第一帧图像匹配的每个特征点的深度
    if (d == 0)
        continue;
    float dd = d / 5000.0;
    Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K); // p1:(X/Z,Y/Z,1)
    // 将相机归一化坐标转换为相机坐标系下的3D组坐标
    pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd, dd));   // (X,Y,Z)第一帧图像的3D坐标（相机坐标系下）
    pts_2d.push_back(keypoints_2[m.trainIdx].pt);          // 第二帧图像的像素坐标
}
```

对于一对匹配点，其中一个点是第一帧图像中的像素点，我们将它的$2D$坐标`keypoints_1[m.queryIdx].pt`转换成了相机坐标系下的$3D$点，并将转换后的$3D$坐标系存入容器`pts_3d`内；另一个点是第二帧图像中的像素点，我们直接获得它的$2D$坐标`keypoints_2[m.trainIdx].pt`，并将其存入容器`pts_2d`内；

## 3.求解PnP

这里使用`EPnP`方法求解`PnP`，直接调用方法即可。

```cpp
// 使用EPnP方法求解PnP
Mat r, t;
solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false, cv::SOLVEPNP_EPNP);
Mat R;
cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
cout << "R=" << endl
    << R << endl;
cout << "t=" << endl
    << t << endl;
```

Parameters:

- objectPoints - 世界坐标系下的控制点的坐标
- imagePoints - 在像素坐标系下对应的控制点的坐标
-  cameraMatrix - 相机的内参矩阵
-  distCoeffs - 相机的畸变系数
-  rvec - 输出的旋转向量
-  tvec - 输出的平移向量
-  flags - 默认使用CV_ITERATIV迭代法

- SolvePnPMethod-选择方法

```cpp
enum SolvePnPMethod {
    SOLVEPNP_ITERATIVE   = 0,
    SOLVEPNP_EPNP        = 1, 
    SOLVEPNP_P3P         = 2, 
    SOLVEPNP_DLS         = 3,
    SOLVEPNP_UPNP        = 4, 
    SOLVEPNP_AP3P        = 5, 
    SOLVEPNP_IPPE        = 6, 
    SOLVEPNP_IPPE_SQUARE = 7,                           
#ifndef CV_DOXYGEN
    SOLVEPNP_MAX_COUNT 
#endif
};
```

打印输出：

```c#
R=
[0.9978745555297591, -0.05102729297915373, 0.04052883908410459;
 0.04983267620066928, 0.9983081506312504, 0.02995898472731967;
 -0.04198899628432293, -0.02787564805402974, 0.9987291286613218]
t=
[-0.1273034869580363;
 -0.01157187487421139;
 0.05667408337434332]
```

## 4.使用BA优化

在使用 g2o 之前，我们要把问题建模成一个最小二乘的图优化问题，节点和边的选择：

1. **节点：**第二个相机的位姿节点 $ξ ∈ se(3)$，以及所有特征点的空间位置 $P ∈ \mathbb{R}^3 $。

2. **边：**每个$ 3D$ 点在第二个相机中的投影，以观测方程来描述：
$$
  z_j = h(ξ, P_j ).
$$

由于第一个相机位姿固定为零，我们没有把它写到优化变量里，但在习题中，我希望你能够把第一个相机的位姿与观测也考虑进来。现在我们根据一组 $3D $点和第二个图像中的 $2D $投影，估计第二个相机的位姿。所以我们把第一个相机画成虚线，表明我们不希望考虑它。

![image-20220326112157782](image-20220326112157782.png)

```cpp
// 使用BA优化
void bundleAdjustment(
    const vector<cv::Point3f> &points_3d,
    const vector<cv::Point2f> &points_2d,
    const Mat &K,
    const Mat &R,
    const Mat &t)
{
    // 构建图优化，先设定g2o
    // 每个误差项优化变量维度为6(即为se3李代数的维数，前三维为平移，后三维为旋转)
    // 误差值维度为3(每个3D点在第二个相机中的投影)
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
    // 实例化线性方程求解器
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
    // 实例化矩阵块求解器
    Block *solver_ptr = new Block(unique_ptr<Block::LinearSolverType>(linearSolver));
    // 梯度下降方法，从GN(高斯牛顿), LM（列文伯格）, DogLeg 中选
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(unique_ptr<Block>(solver_ptr));

    g2o::SparseOptimizer optimizer; // 图模型
    optimizer.setAlgorithm(solver); //设置求解器（使用LM方法）
    optimizer.setVerbose(true);     // 打开调试输出

    // 1.设置节点
    // 优化第二个相机的位姿
    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
    Eigen::Matrix3d R_mat;
    R_mat << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(
        R_mat,
        Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))));
    optimizer.addVertex(pose);
    // 优化所有特征点的空间位置 P
    int index = 1;
    for (const Point3f p : points_3d)
    {
        g2o::VertexPointXYZ *point = new g2o::VertexPointXYZ();
        point->setId(index++);
        point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
        point->setMarginalized(true); // g2o 中必须设置 marg 参见第十讲内容
        optimizer.addVertex(point);
    }
    // 准备相机参数
    g2o::CameraParameters *camera = new g2o::CameraParameters(
        K.at<double>(0, 0), Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)), 0);
    camera->setId(0);
    optimizer.addParameter(camera);

    // 2.设置边(每个 3D 点在第二个相机中的投影)
    index = 1;
    for (const Point2f p : points_2d)
    {
        g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId(index);
        edge->setVertex(0, dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(index)));
        edge->setVertex(1, pose);
        edge->setMeasurement(Eigen::Vector2d(p.x, p.y));
        edge->setParameterId(0, 0);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }
    // 3.优化
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    cout << "after optimization: \n"
         << "T= \n"
         << Eigen::Isometry3d(pose->estimate()).matrix() << endl;
}
```

## 主函数

```cpp
int main(int argc, char **argv)
{
    if (argc != 5)
    {
        cout << "请输入两张彩色图以及对应的深度图：img1 img2 depth1 depth2" << endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    // 1. 特征匹配
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "图一找到" << keypoints_1.size() << "个关键点" << endl;
    cout << "图二找到" << keypoints_2.size() << "个关键点" << endl;
    cout << "筛选后一共" << matches.size() << "组匹配点" << endl;

    // 建立3D点
    Mat d1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED); // 深度图为16位无符号数，单通道图像
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    //提取特征点的深度值：
    for (DMatch m : matches)
    {
        //取得匹配点的深度，queryIdx(表示匹配点的索引)查询描述子索引，pt关键点的坐标 (y行地址)[x列地址]
        ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        if (d == 0)
            continue;
        float dd = d / 5000.0;
        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K); // p1:(X/Z,Y/Z,1)
        // 将相机归一化坐标转换为相机坐标系下的3D组坐标
        pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd, dd)); // (X,Y,Z)第一帧图像的3D坐标（相机坐标系下）
        pts_2d.push_back(keypoints_2[m.trainIdx].pt);        // 第二帧图像的像素坐标
    }
    cout << "3d-2d pairs: " << pts_3d.size() << endl;

    // 使用EPnP方法求解PnP
    Mat r, t;
    solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false, cv::SOLVEPNP_EPNP);
    Mat R;
    cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
    cout << "R=" << endl
         << R << endl;
    cout << "t=" << endl
         << t << endl;

    // 使用BA优化
    bundleAdjustment(pts_3d, pts_2d, K, R, t);

    return 0;
}
```

CMakeLists.txt

```cmake
cmake_minimum_required( VERSION 2.8 )
project( pose_estimation_3d2d )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

# 添加cmake模块以使用g2o
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV 3.1 REQUIRED )
# find_package( OpenCV REQUIRED ) # use this if in OpenCV2 
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    "/usr/include/eigen3/"
)

add_executable( pose_estimation_3d2d pose_estimation_3d2d.cpp )
target_link_libraries( pose_estimation_3d2d 
   ${OpenCV_LIBS}
   ${CSPARSE_LIBRARY}
   g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension g2o_types_slam3d ${CSPARSE_LIBRARY}
)
```

打印输出：

```cpp
max_dist: 94
min_dist: 4
图一找到500个关键点
图二找到500个关键点
筛选后一共79组匹配点
3d-2d pairs: 75
R=
[0.9978745555297591, -0.05102729297915373, 0.04052883908410459;
 0.04983267620066928, 0.9983081506312504, 0.02995898472731967;
 -0.04198899628432293, -0.02787564805402974, 0.9987291286613218]
t=
[-0.1273034869580363;
 -0.01157187487421139;
 0.05667408337434332]
iteration= 0     chi2= 0.003377  time= 0.000995841       cumTime= 0.000995841    edges= 75       schur= 1        lambda= 78.129155         levenbergIter= 1
iteration= 1     chi2= 0.000000  time= 5.9011e-05        cumTime= 0.00105485     edges= 75       schur= 1        lambda= 52.086103         levenbergIter= 1
iteration= 2     chi2= 0.000000  time= 6.0375e-05        cumTime= 0.00111523     edges= 75       schur= 1        lambda= 34.724069         levenbergIter= 1
iteration= 3     chi2= 0.000000  time= 2.4488e-05        cumTime= 0.00113971     edges= 75       schur= 1        lambda= 23.149379         levenbergIter= 1
iteration= 4     chi2= 0.000000  time= 2.3559e-05        cumTime= 0.00116327     edges= 75       schur= 1        lambda= 15.432920         levenbergIter= 1
iteration= 5     chi2= 0.000000  time= 2.3493e-05        cumTime= 0.00118677     edges= 75       schur= 1        lambda= 10.288613         levenbergIter= 1
iteration= 6     chi2= 0.000000  time= 2.3716e-05        cumTime= 0.00121048     edges= 75       schur= 1        lambda= 6.859075  levenbergIter= 1
iteration= 7     chi2= 0.000000  time= 2.3546e-05        cumTime= 0.00123403     edges= 75       schur= 1        lambda= 4.572717  levenbergIter= 1
iteration= 8     chi2= 0.000000  time= 0.000333687       cumTime= 0.00156772     edges= 75       schur= 1        lambda= 1227479343.475617    levenbergIter= 7
after optimization: 
T= 
  0.997852 -0.0508704  0.0412729  -0.128668
 0.0496577   0.998319  0.0298962 -0.0113707
-0.0427243 -0.0277825   0.998701  0.0581576
         0          0          0          1
```

迭代 8轮后，$LM $发现优化目标函数接近不变，于是停止了优化。我们输出了最后得到位姿变换矩阵$ T$ ，对比之前直接做 $PnP $的结果，大约在小数点后第三位发生了一些变化。这主要是由于我们同时优化了特征点和相机位姿导致的。

$Bundle Adjustment $是一种通用的做法。它可以不限于两个图像。我们完全可以放入多个图像匹配到的位姿和空间点进行迭代优化，甚至可以把整个$ SLAM$ 过程放进来。那种做法规模较大，主要在后端使用。在前端，我们通常考虑局部相机位姿和特征点的小型 $Bundle Adjustment $问题，希望实时对它进行求解和优化。
