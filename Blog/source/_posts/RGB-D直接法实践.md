---
title: ch8_RGB-D直接法实践
tags: [SLAM实践]
date: {{ date }}
comments: true
mathjax: true
categories: SLAM十四讲
---

{%cq%}

此次实践使用RGB-D相机主要是因为它省略掉了深度的恢复部分，如果使用单目相机，则需要进行深度的恢复，在之前的实践内容中，我们使用单目相机主要是采用基于特征点的深度恢复，后面会介绍相应的基于块匹配的深度恢复。

{%endcq%}

<!-- more -->

# 一、直接法

直接法根据空间点$P$的来源不同，对应不同的直接法。

- 稀疏直接法

若$P$来自于稀疏关键点，我们称之为稀疏直接法。通常我们使用数百个至上千个关键点，并且像 $L-K $光流那样，假设它周围像素也是不变的。这种稀疏直接法不必计算描述子，并且只使用数百个像素，因此速度最快，但只能计算稀疏的重构。

- 半稠密（Semi-Dense）的直接法

如果像素梯度为零，整一项雅可比就为零，不会对计算运动增量有任何贡献。因此，可以考虑只使用带有梯度的像素点，舍弃像素梯度不明显的地方。
$$
J=-\frac{\partial I_2}{\partial u}
\frac{\partial u}{\partial \delta \xi}
$$

- 稠密直接法

稠密重构需要计算所有像素（一般几十万至几百万个），因此多数不能在现有的 CPU 上实时计算，需要 GPU 的加速。但是，如前面所讨论的，梯度不明显的点，在运动估计中不会有太大贡献，在重构时也会难以估计位置。

> 稀疏到稠密重构，都可以用直接法来计算。它们的计算量是逐渐增长的。**稀疏方法可以快速地求解相机位姿，而稠密方法可以建立完整地图。**具体使用哪种方法，需要视机器人的应用环境而定。特别地，在低端的计算平台上，稀疏直接法可以做到非常快速的效果，适用于实时性较高且计算资源有限的场合

# 二、稀疏直接法

由于求解直接法最后等价于求解一个优化问题，因此我们可以使用$ g2o$ 或$ Ceres $这些优化库帮助我们求解。本节以 $g2o $为例设计实验，在使用 $g2o$之前，需要把直接法抽象成一个图优化问题。显然，直接法是由以下顶点和边组成的：

- 优化变量

优化变量为**一个相机位姿**，因此需要一个位姿顶点。由于我们在推导中使用了李代数，故程序中使用李代数表达的 $SE(3) $位姿顶点。可以使用“Ver-texSE3Expmap”作为相机位姿。

- 误差项

误差项为单个像素的**光度误差**。由于整个优化过程中 $I_1 (p_1 ) $保持不变，我们可以把它当成一个固定的预设值，然后调整相机位姿，使 $I_2 (p_2 )$ 接近这个值。于是，这种边只连接一个顶点，为一元边。由于 $g2o $中本身没有计算光度误差的边，我们需要自己定义一种新的边。

## 2.1 构建直接法的边

直接法的边表示的光度误差，它是一个一元边，定义边的格式之前已经叙述，主要是对`virtual void computeError()`和`virtual void linearizeOplus()`两个函数进行重写。我们分别介绍。

### 光度误差类的定义

```cpp
// 定义直接法的边(光度误差的边),它是一个一元边
class EdgeSE3ProjectDirect : public BaseUnaryEdge<1, double, VertexSE3Expmap>
{
public:
    // 结构体包含eigen成员必须进行宏定义 EIGEN_MAKE_ALIGNED_OPERATOR_NEW, 保证内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3ProjectDirect(Eigen::Vector3d point, float fx, float fy, float cx, float cy, cv::Mat *image) : x_world_(point), fx_(fx), fy_(fy), cx_(cx), cy_(cy), image_(image)
    {
    }
    // 在添加边之前需要设定好该边连接的节点的索引以及观测值和信息矩阵，
    // 这个函数则是使用该边连接的节点和观测值来计算误差
    virtual void computeError(){}
    
    // 计算雅克比矩阵，这个函数是可选的，如果给出了则进行解析求导，不给则进行数值求导
    virtual void linearizeOplus(){}

    // 存盘和读盘：留空
    virtual boolread(istream &in){}
    virtual bool write(ostream &out) const {}

protected:
    // 从参考图像中获取灰度值（双线性插值）
    inline float getPixelValue(float x, float y){}

public:
    Eigen::Vector3d x_world_;                 // 世界坐标系中的3D点
    float cx_ = 0, cy_ = 0, fx_ = 0, fy_ = 0; // 相机内参
    cv::Mat *image_ = nullptr;                //参考图像
};
```

我们的边继承自 `g2o::BaseUnaryEdge`。在继承时，需要在模板参数里填入测量值的维度、类型，以及连接此边的顶点，同时，我们把空间点 P 、相机内参和图像存储在该边的成员变量中。为了让 g2o 优化该边对应的误差，我们需要覆写两个虚函数：用 `computeError()`计算误差值，用` linearizeOplus() `计算雅可比。



### getPixelValue

为了更精细地计算像素亮度，我们要对图像进行插值。我们这里采用了简单的双线性插值，也可以使用更复杂的插值方式，但计算代价可能会变高一些。

所以，相比之前的边的定义，这里多了一个函数` inline float getPixelValue(float x, float y);`，它的作用是从参考图像中获取灰度值，利用的是OpenCV中的双线性插值法，我们先书写它的代码：

```cpp
// 从参考图像中获取灰度值（双线性插值）
inline float getPixelValue(float x, float y)
{
    uchar *data = &image_->data[int(y) * image_->step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
    (1 - xx) * (1 - yy) * data[0] +
    xx * (1 - yy) * data[image_->step] +
    (1 - xx) * yy * data[image_->step + 1]);
}
```

代码解释待理解......

### computeError

由前文可知，误差项为单个像素的**光度误差**。由于整个优化过程中 $I_1 (p_1 ) $保持不变，我们可以把它当成一个固定的预设值，然后调整相机位姿，使 $I_2 (p_2 )$ 接近这个值。`virtual void computeError()`这个函数则是使用该边连接的节点和观测值来计算误差。

程序中的误差计算里，使用了$ I_2 (p_2 ) − I_1 (p_1 )$ 的形式，因此前面的负号可以省去，只需把像素梯度乘以像素到李代数的梯度即可。

```cpp
// 在添加边之前需要设定好该边连接的节点的索引以及观测值和信息矩阵，
// 这个函数则是使用该边连接的节点和观测值来计算误差
virtual void computeError()
{
    const VertexSE3Expmap *v = static_cast<const VertexSE3Expmap *>(_vertices[0]);
    Eigen::Vector3d x_local = v->estimate().map(x_world_); //.map的功能是把世界坐标系下三维点变换到相机坐标系
    // 获得像素坐标
    float x = x_local[0] * fx_ / x_local[2] + cx_; // x(u) = (X * fx)/Z + cx
    float y = x_local[1] * fy_ / x_local[2] + cy_; // y(v) = (Y * fy)/Z + cy
    // 检查该像素点是否在图像内
    if (x - 4 < 0 || (x + 4) > image_->cols || (y - 4) < 0 || (y + 4) > image_->rows)
    {
        _error(0, 0) = 0.0;
        this->setLevel(1);
    }
    else
    {
        _error(0, 0) = getPixelValue(x, y) - _measurement; // 光度误差，像素的亮度误差，或者是灰度值误差
    }
}
```

### linearizeOplus

这个函数主要是计算雅克比矩阵，它是可选的，如果给出了则进行解析求导，不给则进行数值求导。我们直接在直接法的推导中已经介绍了该矩阵的计算，下面是推导结论，因此直接根据公式书写代码即可。

由直接法的推导可知：
$$
\begin{align}
\frac{\partial u}{\partial \delta \xi}&=
\frac{\partial u}{\partial q}
\frac{\partial q}{\partial \delta \xi}=
\begin{bmatrix}
\frac{f_x}{Z}&0&-\frac{f_xX}{Z^{2}}
&-\frac{f_xXY}{Z^{2}}&f_x+\frac{f_xX^2}{Z^{2}}&-\frac{f_xY}{Z}
\\
0&\frac{f_y}{Z}&-\frac{f_yY}{Z^{2}}
&-f_y-\frac{f_yY^{2}}{Z^{2}}&\frac{f_yXY}{Z}&\frac{f_yX}{Z}
\end{bmatrix}
\end{align}
$$
最终的约旦矩阵为：
$$
J=-\frac{\partial I_2}{\partial u}
\frac{\partial u}{\partial \delta \xi}
$$
由于上面的误差使用的是$ I_2 (p_2 ) − I_1 (p_1 )$ 的形式，所以：
$$
J=\frac{\partial I_2}{\partial u}
\frac{\partial u}{\partial \delta \xi}
$$

```cpp
virtual void linearizeOplus()
{
    if (level() == 1)
    {
        _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
        return;
    }
    // VertexSE3Expmap，这个表示李代数的位姿；
    VertexSE3Expmap *vtx = static_cast<VertexSE3Expmap *>(_vertices[0]);
    // 将世界坐标系下的三维点的估计转换成相机坐标系下
    Eigen::Vector3d xyz_trans = vtx->estimate().map(x_world_);
    double x = xyz_trans[0];          // X
    double y = xyz_trans[1];          // Y
    double invz = 1.0 / xyz_trans[2]; // 1/Z
    double invz_2 = invz * invz;      // 1/Z^2

    float u = x * fx_ * invz + cx_; // u = (X*fx)/Z+cx
    float v = v * fy_ * invz + cy_; // v = (Y*fy)/Z+cy

    // 先求投影方程关于变换的导数 ∂u/∂δξ
    Eigen::Matrix<double, 2, 6> jacobian_uv_ksai;
    // g2o 是旋转在前，平移在后；
    jacobian_uv_ksai(0, 0) = -x * y * fx_ * invz_2;
    jacobian_uv_ksai(0, 1) = (1 + x * x * invz_2) * fx_;
    jacobian_uv_ksai(0, 2) = -fx_ * y * invz;
    jacobian_uv_ksai(0, 3) = fx_ * invz;
    jacobian_uv_ksai(0, 4) = 0;
    jacobian_uv_ksai(0, 6) = -fx_ * x * invz_2;

    jacobian_uv_ksai(1, 0) = -(1 + y * y * invz_2) * fy_;
    jacobian_uv_ksai(1, 1) = fy_ * x * y * invz_2;
    jacobian_uv_ksai(1, 2) = fy_ * x * invz;
    jacobian_uv_ksai(1, 3) = 0;
    jacobian_uv_ksai(1, 4) = fy_ * invz;
    jacobian_uv_ksai(1, 5) = -y * invz_2 * fy_;

    // 像素的梯度 ∂I2/∂u
    Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;

    jacobian_pixel_uv(0, 0) = (getPixelValue(u + 1, v) - getPixelValue(u - 1, v)) / 2;
    jacobian_pixel_uv(0, 1) = (getPixelValue(u, v + 1) - getPixelValue(u, v - 1)) / 2;

    // J矩阵 = ∂I2/∂u * ∂u/∂δξ
    _jacobianOplusXi = jacobian_pixel_uv*jacobian_uv_ksai;
}
```

## 2.2 直接法估计相机运动

在使用直接法估计相机运动时，我们用过g2o对相机的位姿进行优化，其中的观测值为世界坐标系下的3D点坐标以及对应的灰度值，因此我们使用一个结构体`Measurement`用来存储观测值，由于需要用到相机坐标系之间的变换，所以还需要写坐标变换相关的函数。

### Measurement结构体

```cpp
// 一次测量的值，包括一个世界坐标系下三维点与一个灰度值
struct Measurement
{
    Measurement(Eigen::Vector3d p, float g) : pos_world(p), grayscale(g) {}

    Eigen::Vector3d pos_world;
    float grayscale;
};
```

### project2Dto3D

```cpp
// 将像素坐标系转换为世界坐标系
inline Eigen::Vector3d project2Dto3D(int x, int y, int d, float fx, float fy, float cx, float cy, float scale)
{
    float zz = float(d) / scale;
    float xx = zz * (x - cx) / fx; // X = Z*(u-cx)/fx
    float yy = zz * (y - cy) / fy; // Y = Z*(u-cy)/fy
    return Eigen::Vector3d(xx, yy, zz);
}
```

### project3Dto2D

```cpp
// 将世界坐标系转换为像素坐标系
inline Eigen::Vector2d project3Dto2D(float x, float y, float z, float fx, float fy, float cx, float cy)
{
    float u = fx * x / z + cx; // u = fx*X/Z + cx
    float v = fy * y / z + cy; // v = fy*Y/Z + cy
    return Eigen::Vector2d(u, v);
}
```

### poseEstimationDirect

这是最主要的一个函数，用直接法估计相机运动，并使用了g2o对位姿进行优化。**相比于特征点法，直接法完全依靠优化来求解相机位姿。**

原理就是以第一个图像为参考帧，然后用直接法求解后续图像的位姿。在参考帧中，对第一张图像提取 FAST 关键点（不需要描述子），并使用直接法估计这些关键点在第二个图像中的位置，以及第二个图像的相机位姿。

```cpp
// 使用直接法估计相机运动(使用非线性BA优化)
bool poseEstimationDirect(
    const vector<Measurement> &measurements, // 测量值：一个世界坐标系下三维点与一个灰度值
    cv::Mat *gray,                          // 灰度图
    Eigen::Matrix3f &K,                     // 相机参数
    Eigen::Isometry3d &Tcw)
{
    // 初始化g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 1>> DirectBlock;
    // 1.创建线性求解器 LinearSolverDense使用dense cholesky分解法
    DirectBlock::LinearSolverType *linearSolver = new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>();
    // 2.创建矩阵块求解器，用上面我们定义的线性求解器LinearSolver来初始化。
    DirectBlock *solver_ptr = new DirectBlock(std::unique_ptr<DirectBlock::LinearSolverType>(linearSolver));
    // 3.创建总求解器solver
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(unique_ptr<DirectBlock>(solver_ptr));
    // 4.创建稀疏优化器
    g2o::SparseOptimizer optimizer; // 创建稀疏优化器
    optimizer.setAlgorithm(solver); // 用前面定义好的求解器作为求解方法：（使用LM方法）
    optimizer.setVerbose(true);     // setVerbose是设置优化过程输出信息用

    // 5.定义图的顶点和边
    // 顶点(待优化变量，这里是相机位姿)
    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
    // 设置待优化位姿旋转(Rotation)(角轴或四元素表示)、平移(Translation)
    pose->setEstimate(g2o::SE3Quat(Tcw.rotation(), Tcw.translation()));
    pose->setId(0);
    optimizer.addVertex(pose);

    // 边(误差，这里是光影误差)
    int id = 1;
    for (Measurement m : measurements)
    {
        EdgeSE3ProjectDirect *edge = new EdgeSE3ProjectDirect(
            m.pos_world,
            K(0, 0), K(1, 1), K(0, 2), K(1, 2), gray);
        edge->setVertex(0, pose);                                      // 定义顶点
        edge->setMeasurement(m.grayscale);                             // 定义观测值
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity()); // 定义协方差矩阵的逆
        edge->setId(id++);
        optimizer.addEdge(edge);
    }
    cout << "图中的边的个数：" << optimizer.edges().size() << endl;
    // 6.开始优化
    optimizer.initializeOptimization();
    optimizer.optimize(30); // 迭代次数
    // 优化后的相机位姿
    Tcw = pose->estimate();
    return true;
}
```

## 2.3 主函数

整个主函数的顺序需要了解

- 首先是一些判断或者一些初始化
- 然后我们对数据集循环操作每一张图片，但是第一张图像为参考，对后续图像和参考图像做直接法
- 对第一张图提取Fast关键点（不需要描述子）
- 使用直接法估计这些关键点在第二个图像中的位置，以及第二个图像的相机位姿
- 最后就是绘制这些特征点

```cpp
int main(int argc, char **argv)
{
    if (argc != 2)
    {
        cout << "usage: useLK path_to_dataset" << endl;
        return 1;
    }

    srand((unsigned int)time(0));
    string path_to_dataset = argv[1];
    string assciate_file = path_to_dataset + "/assciate.txt";

    ifstream fin(assciate_file);
    string rgb_file, depth_file, time_rgb, time_depth;
    cv::Mat color, depth, gray;
    vector<Measurement> measurements;
    // 相机内参
    float cx = 325.5;
    float cy = 253.5;
    float fx = 518.0;
    float fy = 519.0;
    float depth_scale = 1000.0;
    Eigen::Matrix3f K;
    K << fx, 0.f, cx, 0.f, fy, cy, 0.f, 0.f, 1.0f;

    Eigen::Isometry3d Tcw = Eigen::Isometry3d::Identity();
    cv::Mat prev_color;

    // 我们以第一个图像为参考，对后续图像和参考图像做直接法
    for (int index = 0; index < 10; index++)
    {
        cout << "*********** loop " << index << " ************" << endl;
        fin >> time_rgb >> rgb_file >> time_depth >> depth_file;
        color = cv::imread(path_to_dataset + "/" + rgb_file);
        depth = cv::imread(path_to_dataset + "/" + depth_file, -1);

        if (color.data == nullptr || depth.data == nullptr)
            continue; // continue 语句的作用是跳过循环体中剩余的语句而强制进入下一次循环
        // 将BGR格式(color)转换成灰度图片(gray)
        cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);

        // 1.对第一帧提取FAST特征点
        if (index == 0)
        {
            vector<cv::KeyPoint> keypoints;
            cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            // 使用Fast特征点检测，讲检测后的特征点存入容器keypoints中
            detector->detect(color, keypoints);

            // 去掉邻近边缘处的点
            for (auto kp : keypoints)
            {
                if (kp.pt.x < 20 || kp.pt.y < 20 || (kp.pt.x + 20) > color.cols || (kp.pt.y + 20) > color.rows)
                    continue; // continue 语句的作用是跳过循环体中剩余的语句而强制进入下一次循环
                // d指定到depth矩阵的y行第x列个像素
                ushort d = depth.ptr<ushort>(cvRound(kp.pt.y))[cvRound(kp.pt.x)];
                if (d == 0)
                    continue;
                // 将像素坐标系转换为世界坐标系
                Eigen::Vector3d p3p = project2Dto3D(kp.pt.x, kp.pt.y, d, fx, fy, cx, cy, depth_scale);
                // grayscale指定到gray矩阵的y行第x列个像素
                float grayscale = float(gray.ptr(cvRound(kp.pt.y))[cvRound(kp.pt.x)]);
                // 一次测量的值，包括一个世界坐标系下三维点与一个灰度值
                measurements.push_back(Measurement(p3p, grayscale));
            }
            prev_color = color.clone();
            cout<<"add total "<<measurements.size()<<" measurements."<<endl;
            continue;
        }
        // 2.对后续图像使用直接法计算相机运动
        poseEstimationDirect(measurements, &gray, K, Tcw);
        cout << "Tcw=" << Tcw.matrix() << endl;

        // 3.绘制特征点
        cv::Mat img_show(color.rows * 2, color.cols, CV_8UC3);
        prev_color.copyTo(img_show(cv::Rect(0, 0, color.cols, color.rows)));
        color.copyTo(img_show(cv::Rect(0, color.rows, color.cols, color.rows)));
        for (Measurement m : measurements)
        {
            if (rand() > RAND_MAX / 5)
                continue;
            Eigen::Vector3d p = m.pos_world;
            Eigen::Vector2d pixel_prev = project3Dto2D(p(0, 0), p(1, 0), p(2, 0), fx, fy, cx, cy);
            Eigen::Vector3d p2 = Tcw * m.pos_world;
            Eigen::Vector2d pixel_now = project3Dto2D(p2(0, 0), p2(1, 0), p2(2, 0), fx, fy, cx, cy);
            if (pixel_now(0, 0) < 0 || pixel_now(0, 0) >= color.cols || pixel_now(1, 0) < 0 || pixel_now(1, 0) >= color.rows)
                continue;

            float b = 255 * float(rand()) / RAND_MAX;
            float g = 255 * float(rand()) / RAND_MAX;
            float r = 255 * float(rand()) / RAND_MAX;
            cv::circle(img_show, cv::Point2d(pixel_prev(0, 0), pixel_prev(1, 0)), 8, cv::Scalar(b, g, r), 2);
            cv::circle(img_show, cv::Point2d(pixel_now(0, 0), pixel_now(1, 0) + color.rows), 8, cv::Scalar(b, g, r), 2);
            cv::line(img_show, cv::Point2d(pixel_prev(0, 0), pixel_prev(1, 0)), cv::Point2d(pixel_now(0, 0), pixel_now(1, 0) + color.rows), cv::Scalar(b, g, r), 1);
        }
        cv::imshow("result", img_show);
        cv::waitKey(0);
    }

    return 0;
}
```

CMakeLists.txt

```cmake
cmake_minimum_required( VERSION 2.8 )
project( directMethod )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

# 添加cmake模块路径
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( G2O )
include_directories( ${G2O_INCLUDE_DIRS} ) 

include_directories( "/usr/include/eigen3" )

set( G2O_LIBS 
    g2o_core g2o_types_sba g2o_solver_csparse g2o_stuff g2o_csparse_extension 
)

add_executable( direct_sparse direct_sparse.cpp )
target_link_libraries( direct_sparse ${OpenCV_LIBS} ${G2O_LIBS} )

add_executable( direct_semidense direct_semidense.cpp )
target_link_libraries( direct_semidense ${OpenCV_LIBS} ${G2O_LIBS} )
```

输出

```css
*********** loop 0 ************
add total 1402 measurements.
*********** loop 1 ************
图中的边的个数：1402
iteration= 0     chi2= 7403254.830490    time= 0.00082488        cumTime= 0.00082488     edges= 1402     schur= 0        lambda= 60236876.490033        levenbergIter= 5
iteration= 1     chi2= 7403254.830490    time= 0.00102553        cumTime= 0.00185041     edges= 1402     schur= 0        lambda= 2119396675894340747264.000000  levenbergIter= 9
Tcw= 
    0.999996   0.00288273  2.64449e-05 -0.000928174
 -0.00288272     0.999996 -0.000436968 -3.76331e-05
-2.77045e-05   0.00043689            1   0.00103318
           0            0            0            1
*********** loop 2 ************
图中的边的个数：1402
iteration= 0     chi2= 7759380.788781    time= 0.00174707        cumTime= 0.00174707     
......
iteration= 9     chi2= 7758019.671470    time= 0.000755714       cumTime= 0.00772499     edges= 1402     schur= 0        lambda= 3783063956763384479744.000000  levenbergIter= 5
Tcw= 
    0.999996   0.00288243  2.77591e-05 -0.000928009
 -0.00288242     0.999996 -0.000441154 -3.82145e-05
-2.90306e-05  0.000441072            1   0.00103304
           0            0            0            1
*********** loop 3 ************
图中的边的个数：1402
iteration= 0     chi2= 7824244.109174    time= 0.00174816        cumTime= 0.00174816     edges= 1402     schur= 0        lambda= 6220200914927.365234   levenbergIter= 8
......
iteration= 8     chi2= 7819308.542160    time= 0.0010889         cumTime= 0.00689116     edges= 1402     schur= 0        lambda= 3127198551927549329408.000000  levenbergIter= 7
Tcw= 
    0.999996   0.00289109  2.80266e-05 -0.000927608
 -0.00289108     0.999996 -0.000451442 -3.83751e-05
-2.93317e-05  0.000451359            1   0.00103126
           0            0            0            1
*********** loop 4 ************
图中的边的个数：1402
iteration= 0     chi2= 8430224.068489    time= 0.0017704         cumTime= 0.0017704      edges= 1402     schur= 0        lambda= 14477120284239.218750  levenbergIter= 8
......
iteration= 10    chi2= 8429345.097617    time= 0.000582781       cumTime= 0.00791508     edges= 1402     schur= 0        lambda= 25272065502643142656.000000    levenbergIter= 4
Tcw= 
    0.999996    0.0028903  2.74361e-05 -0.000927784
 -0.00289029     0.999996  -0.00044829 -3.78331e-05
-2.87317e-05  0.000448209            1   0.00103136
           0            0            0            1
*********** loop 5 ************
图中的边的个数：1402
iteration= 0     chi2= 8454586.253220    time= 0.00216226        cumTime= 0.00216226     edges= 1402     schur= 0        lambda= 11915390830915310583808.000000         levenbergIter= 10
Tcw= 
    0.999996    0.0028903  2.74361e-05 -0.000927784
 -0.00289029     0.999996  -0.00044829 -3.78331e-05
-2.87317e-05  0.000448209            1   0.00103136
           0            0            0            1
*********** loop 6 ************
图中的边的个数：1402
iteration= 0     chi2= 8159516.143149    time= 0.00214553        cumTime= 0.00214553     edges= 1402     schur= 0        lambda= 7207697814143619825664.000000  levenbergIter= 10
Tcw= 
    0.999996    0.0028903  2.74361e-05 -0.000927784
 -0.00289029     0.999996  -0.00044829 -3.78331e-05
-2.87317e-05  0.000448209            1   0.00103136
           0            0            0            1
*********** loop 7 ************
图中的边的个数：1402
iteration= 0     chi2= 8203720.110076    time= 0.0021112         cumTime= 0.0021112      edges= 1402     schur= 0        lambda= 21225209233812976828416.000000         levenbergIter= 10
Tcw= 
    0.999996    0.0028903  2.74361e-05 -0.000927784
 -0.00289029     0.999996  -0.00044829 -3.78331e-05
-2.87317e-05  0.000448209            1   0.00103136
           0            0            0            1
*********** loop 8 ************
图中的边的个数：1402
iteration= 0     chi2= 8054487.520687    time= 0.00161749        cumTime= 0.00161749     edges= 1402     schur= 0        lambda= 306330210612.899292    levenbergIter= 7
......
iteration= 5     chi2= 8047353.368341    time= 0.00105102        cumTime= 0.00563404     edges= 1402     schur= 0        lambda= 4158192944542638931968.000000  levenbergIter= 6
Tcw= 
    0.999996   0.00290379  2.91578e-05 -0.000927602
 -0.00290378     0.999996 -0.000470562 -4.13493e-05
-3.05241e-05  0.000470476            1   0.00103238
           0            0            0            1
*********** loop 9 ************
```

![image-20220329164608300](image-20220329164608300.png)

# 三、半稠密直接法

我们很容易就能把程序拓展成半稠密的直接法形式。对参考帧中，先提取梯度较明显的像素，然后用直接法，以这些像素为图优化边，来估计相机运动。对先前的程序做如下的修改：

```cpp
// 1.对第一帧提取FAST特征点
if (index == 0)
{
    // select the pixels with high gradiants 
    for ( int x=10; x<gray.cols-10; x++ )
        for ( int y=10; y<gray.rows-10; y++ )
        {
            Eigen::Vector2d delta (
            gray.ptr<uchar>(y)[x+1] - gray.ptr<uchar>(y)[x-1], 
            gray.ptr<uchar>(y+1)[x] - gray.ptr<uchar>(y-1)[x]
            );
            if ( delta.norm() < 50 )
                continue;
            ushort d = depth.ptr<ushort> (y)[x];
            if ( d==0 )
                continue;
            Eigen::Vector3d p3d = project2Dto3D ( x, y, d, fx, fy, cx, cy, depth_scale );
            float grayscale = float ( gray.ptr<uchar> (y) [x] );
            measurements.push_back ( Measurement ( p3d, grayscale ) );
        }
        prev_color = color.clone();
        cout<<"add total "<<measurements.size()<<" measurements."<<endl;
        continue;
}
```

输出：

```CSS
*********** loop 0 ************
add total 12556 measurements.
*********** loop 1 ************
图中的边的个数：12556
.....
iteration= 19    chi2= 71515789.320224   time= 0.00527771        cumTime= 0.0867877      edges= 12556    schur= 0        lambda= 4022565853557871345664.000000  levenbergIter= 4
Tcw= 
    0.999999 -1.99501e-06  -0.00124229    0.0166827
 3.02563e-06            1  0.000829608   -0.0119843
  0.00124229 -0.000829611     0.999999   -0.0118238
           0            0            0            1
....
*********** loop 9 ************
```

![image-20220330165208010](/home/alvin/Documents/lukeyalvin.github.io/Blog/source/_posts/RGB-D%E7%9B%B4%E6%8E%A5%E6%B3%95%E5%AE%9E%E8%B7%B5/image-20220330165208010.png)

