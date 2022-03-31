---
title: ch7_求解ICP并使用BA优化
tags: [SLAM实践]
date: {{ date }}
comments: true
mathjax: true
categories: SLAM十四讲
---

{%cq%}

$ICP:$迭代最近点$（Iterative Closest Point）$求解。读者应该注意到，$3D-3D $位姿估计问题中，并没有出现相机模型，也就是说，仅考虑两组 $3D $点之间的变换时，和相机并没有关系。因此，在激光 $SLAM $中也会碰到 $ICP$，不过由于激光数据特征不够丰富，我们无从知道两个点集之间的匹配关系，只能认为距离最近的两个点为同一个，所以这个方法称为迭代最近点。而在视觉中，特征点为我们提供了较好的匹配关系，所以整个问题就变得更简单了。

{%endcq%}

<!-- more -->

和 $PnP$ 类似，$ICP $的求解也分为两种方式：利用线性代数的求解（主要是$ SVD$），以及利用非线性优化方式的求解（类似于 $Bundle Adjustment$）。

# 求解ICP

## SVD方法

### 构建最小二乘

假设我们有一组配对好的 3D 点（比如我们对两个 RGB-D 图像进行了匹配）：
$$
P = {p_1 , . . . , p_n } \\ P^′= {p^′_1 , . . . , p^′_n }
$$
现在，想要找一个欧氏变换 $R, t，$使得：
$$
∀i, p_i = Rp^′_i + t.
$$
定义第 $i$对点的误差项：
$$
e_i=p_i - (Rp^′_i + t).
$$
构建最小二乘问题:
$$
\underset{R,t}{min}J=\frac{1}{2}\sum^n_{i=1}\parallel (p_i - (Rp^′_i + t)) \parallel^2_2
$$

### 求解最小二乘

首先，定义两组点的质心(注意质心是没有下标)：
$$
p=\frac{1}{n}\sum^n_{i=1}(p_i)，p^{\prime}=\frac{1}{n}\sum^n_{i=1}(p^{\prime}_i)
$$
在误差函数中，我们作如下的处理：
$$
\begin{align}
\frac{1}{2}\sum^n_{i=1}\parallel (p_i - (Rp^{\prime}_i + t)) \parallel^2
&=\frac{1}{2}\sum^n_{i=1}\parallel p_i - Rp^{\prime}_i - t-p+Rp^{\prime}+p-Rp^{\prime} \parallel^2
\\&= \frac{1}{2}\sum^n_{i=1}\parallel (p_i -p- R(p^{\prime}_i-p^{\prime}) )+(p- Rp^{\prime}+t) \parallel^2
\\&= \frac{1}{2}\sum^n_{i=1}(\parallel p_i -p- R(p^{\prime}_i-p^{\prime}) \parallel^2+
\parallel p- Rp^{\prime}+t\parallel^2
\\&+2(p_i -p-R(p^{\prime}_i-p^{\prime})^T(p-Rp^{\prime}-t))
\end{align}
$$
其中：
$$
\begin{align}
p_i-p-R(p^{\prime}_i-p^{\prime})&=(Rp^{\prime}_i+t)-(Rp^{\prime}+t)-R(p^{\prime}_i-p^{\prime})\\
&=Rp^{\prime}_i+t-Rp^{\prime}-t-Rp^{\prime}_i+Rp^{\prime}\\
&=0
\end{align}
$$
所以：
$$
\begin{align}
\underset{R,t}{min}J&=\frac{1}{2}\sum^n_{i=1}\parallel (p_i - (Rp^′_i + t)) \parallel^2\\
&=\frac{1}{2}\sum^n_{i=1}\parallel p_i -p- R(p^{\prime}_i-p^{\prime}) \parallel^2+
\parallel p- Rp^{\prime}+t\parallel^2
\end{align}
$$
仔细观察左右两项，我们发现左边只和旋转矩阵 R 相关，而右边既有 R 也有 t，但只和质心相关。只要我们获得了 R，令第二项为零就能得到 t。于是，ICP 可以分为以下三个步骤求解：

1.计算两组点的质心位置 $p$, $p^{\prime}$，然后计算每个点的去质心坐标：
$$
q_i = p_i − p,\\
q_i^{\prime} = p_i^{\prime} − p^{\prime}  .
$$
2.根据以下优化问题计算旋转矩阵：
$$
R^{*}=arg\underset{R}{min}\frac{1}{2}\sum^n_{i=1}\parallel q_i - Rq_i^{\prime} \parallel^2
$$
3.根据第二步的 $R$，计算 $t$：
$$
t^∗ = p − Rp^{\prime} .
$$
计算$R$：
$$
\frac{1}{2}\sum^n_{i=1}\parallel q_i - Rq_i^{\prime} \parallel^2
=\frac{1}{2}\sum^n_{i=1}q_i^{T}q_i+q_i^{\prime T}R^TRq_i^{\prime}-2q^T_iRq_i^{\prime}
$$
第一项和$ R$ 无关，第二项由于 $R^T R = I$，亦与 $R $无关。因此，实际上优化目标函数变为：
$$
\sum^n_{i=1}-q_i^{T}Rq_i^{\prime}=\sum^n_{i=1}-tr(Rq_i^{\prime}q_i^{T})=-tr(R\sum^n_{i=1}q_i^{\prime}q_i^{T})
$$
定义：
$$
W=\sum^n_{i=1}q_i^{\prime}q_i^{T}
$$
$W $是一个$ 3 × 3 $的矩阵，对$ W$ 进行 $SVD$ 分解，得：
$$
W = UΣV^T
$$
$Σ $为奇异值组成的对角矩阵，对角线元素从大到小排列，而 $U $和$ V$ 为正交矩阵。当 $W$ 满秩时，$R$ 为：
$$
R = UV^T
$$
解得 $R$后,按照$t^∗ = p − Rp^{\prime} .$求解 t 即可

## 非线性优化方法

求解$ ICP$ 的另一种方式是使用非线性优化，以迭代的方式去找最优值。该方法和我们前面讲述的 $PnP $非常相似。以李代数表达位姿时，目标函数可以写成：
$$
\underset{\xi}{min}=\frac{1}{2}\sum^n_{i=1}\parallel (p_i - exp(\xi^{\land})p^{\prime}_i) \parallel^2_2
$$

单个误差项关于位姿导数已经在前面推导过了，使用李代数扰动模型即可:
$$
\frac{\partial e}{\partial \delta \xi}=-(exp(\xi^{\land}p^{\prime}_i))^{⊙}
$$

用非线性优化来计算$ ICP$不仅考虑相机的位姿，同时会优化 $3D$点的空间位置。对我们来说，RGB-D 相机每次可以观测到路标点的三维位置，从而产生一个$ 3D$ 观测数据。不过，由于$ g2o/sba$ 中没有提供 $3D$ 到 $3D$ 的边，而我们又想使用$ g2o/sba $中李代数实现的位姿节点，所以最好的方式是自定义一种这样的边，并向$ g2o$ 提供解析求导方式。



# 代码实现

## 1. 特征点匹配

```cpp
// 1.特征点匹配
void find_feature_matches(
    const Mat &img_1,
    const Mat &img_2,
    std::vector<KeyPoint> &keypoints_1,
    std::vector<KeyPoint> &keypoints_2,
    std::vector<DMatch> &matches)
{
    //-- 初始化
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
    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        double dist = match[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }
    cout << "max_dist: " << max_dist << endl;
    cout << "min_dist: " << min_dist << endl;

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (match[i].distance <= max(2 * min_dist, 30.0))
        {
            matches.push_back(match[i]);
        }
    }
}
```

## 2.建立3D点

```cpp
void build_3d_points(
    const Mat &depth_1,
    const Mat &depth_2,
    const Mat &K,
    const vector<KeyPoint> &keypoints_1,
    const vector<KeyPoint> &keypoints_2,
    const vector<DMatch> matches,
    vector<Point3f> &pts1,
    vector<Point3f> &pts2)
{
    //提取特征点的深度值：
    for (DMatch m : matches)
    {
        //取得匹配点的深度，queryIdx(表示匹配点的索引)查询描述子索引，pt关键点的坐标 (y行地址)[x列地址]
        ushort d1 = depth_1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        ushort d2 = depth_2.ptr<unsigned short>(int(keypoints_2[m.trainIdx].pt.y))[int(keypoints_2[m.trainIdx].pt.x)];
        if (d1 == 0 || d2 == 0)
            continue;
        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);// p1:(X/Z,Y/Z,1)
        Point2d p2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);// p2:(X/Z,Y/Z,1)
        float dd1 = float(d1) / 5000.0;
        float dd2 = float(d2) / 5000.0;
        // 将相机归一化坐标转换为相机坐标系下的3D组坐标
        pts1.push_back(Point3f(p1.x * dd1, p1.y * dd1, dd1));// (X,Y,Z)第一帧图像的3D坐标（相机坐标系下）
        pts2.push_back(Point3f(p2.x * dd2, p2.y * dd2, dd2));// (X,Y,Z)第二帧图像的3D坐标（相机坐标系下）
    }
    cout << "3d-3d pairs: " << pts1.size() << endl;
}
```

## 3.SVD求解ICP

```cpp
// 3.SVD求解ICP
void pose_estimation_3d3d(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R, Mat &t)
{
    Point3f p1, p2;

    int N = pts1.size(); // N = 72;
    for (int i = 0; i < N; i++)
    {
        p1 += pts1[i]; // p1 为pts1坐标各维之和
        p2 += pts2[i]; // p2 为pts2坐标各维之和
    }

    p1 = Point3f(Vec3f(p1) / N); // 取平均得到质心p1
    p2 = Point3f(Vec3f(p2) / N); // 取平均得到质心p2

    vector<Point3f> q1(N), q2(N); // remove the center
    for (int i = 0; i < N; i++)
    {
        q1[i] = pts1[i] - p1; // 计算每个点的去质心坐标q1
        q2[i] = pts2[i] - p2; // 计算每个点的去质心坐标q2
    }
    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++)
    {
        // w = sum(q1*q2^T)
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    cout << "W=" << W << endl;
    // 对W进行SVD分解
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU(); // 正交矩阵U
    Eigen::Matrix3d V = svd.matrixV(); // 正交矩阵V
    if (U.determinant() * V.determinant() < 0)
    {
        for (int x = 0; x < 3; ++x)
        {
            U(x, 2) *= -1;
        }
    }
    cout << "U=" << U << endl;
    cout << "V=" << V << endl;
    Eigen::Matrix3d R_ = U * (V.transpose());
    Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

    R = (Mat_<double>(3, 3) << 
         R_(0, 0), R_(0, 1), R_(0, 2),
         R_(1, 0), R_(1, 1), R_(1, 2),
         R_(2, 0), R_(2, 1), R_(2, 2));
    t = ( Mat_<double> ( 3,1 ) << t_ ( 0,0 ), t_ ( 1,0 ), t_ ( 2,0 ) );
}
```

## 4.BA优化

### 自定义3D-3D的边

由于 g2o/sba 中没有提供 3D 到 3D 的边，而我们又想使用 g2o/sba 中李代数实现的位姿节点，所以最好的方式是自定义一种这样的边，并向 g2o 提供解析求导方式。

 $J$矩阵:
$$
\begin{bmatrix}
0 	&-z &y 	&-1 &0 	&0 \\
z 	&0 	&-x &0 	&-1 &0 \\
-y 	&x  &0 	&0 	&0 	&-1
\end{bmatrix}
$$
自定义 3D 到 3D 的边：

```cpp
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>
{
public:
    // 结构体包含eigen成员必须进行宏定义 EIGEN_MAKE_ALIGNED_OPERATOR_NEW, 保证内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // 构造函数
    EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d &point) : _point(point) {}

    // 再添加边之前需要设定好该边连接的节点的索引以及观测值和信息矩阵，
    // 这个函数则是使用该边连接的节点和观测值来计算误差
    virtual void computeError()
    {
        const g2o::VertexSE3Expmap *pose = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        // measurement is p, point is p'
        _error = _measurement - pose->estimate().map(_point); // 重投影误差
    }

    // 计算雅克比矩阵，这个函数是可选的，如果给出了则进行解析求导，不给则进行数值求导
    virtual void linearizeOplus()
    {
        // VertexSE3Expmap，这个表示李代数的位姿；
        g2o::VertexSE3Expmap *pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat T(pose->estimate());
        Eigen::Vector3d xyz_trans = T.map(_point); // .map的功能是把世界坐标系下三维点变换到相机坐标系
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        //_jacobianOplusXi是误差关于世界坐标系下坐标点的偏导
        _jacobianOplusXi(0,0) = 0;
        _jacobianOplusXi(0,1) = -z;
        _jacobianOplusXi(0,2) = y;
        _jacobianOplusXi(0,3) = -1;
        _jacobianOplusXi(0,4) = 0;
        _jacobianOplusXi(0,5) = 0;

        _jacobianOplusXi(1,0) = z;
        _jacobianOplusXi(1,1) = 0;
        _jacobianOplusXi(1,2) = -x;
        _jacobianOplusXi(1,3) = 0;
        _jacobianOplusXi(1,4) = -1;
        _jacobianOplusXi(1,5) = 0;

        _jacobianOplusXi(2,0) = -y;
        _jacobianOplusXi(2,1) = x;
        _jacobianOplusXi(2,2) = 0;
        _jacobianOplusXi(2,3) = 0;
        _jacobianOplusXi(2,4) = 0;
        _jacobianOplusXi(2,5) = -1;
    }
    // 存盘和读盘：留空
    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {}

protected:
    Eigen::Vector3d _point;
};
```

### BA优化

```cpp
// 4.BA优化
void bundleAdjustment(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R, Mat &t)
{
    // 1.初始化
    // 每个误差项优化变量维度为6(即为se3李代数的维数，前三维为平移，后三维为旋转)
    // 误差值维度为3(每个3D点在第二个相机中的投影)
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
    // 实例化线性方程求解器:使用eigen中sparse Cholesky 求解
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>();
    // 实例化矩阵块求解器
    Block *solver_ptr = new Block(unique_ptr<Block::LinearSolverType>(linearSolver));
    // 梯度下降方法，GN(高斯牛顿)418133364
    g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(unique_ptr<Block>(solver_ptr));

    g2o::SparseOptimizer optimizer; // 创建稀疏优化器
    optimizer.setAlgorithm(solver); // 用前面定义好的求解器作为求解方法：（使用GN(高斯牛顿)方法）
    optimizer.setVerbose(true);     // setVerbose是设置优化过程输出信息用

    // 2.构建顶点
    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap(); // 相机位姿
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0)));
    optimizer.addVertex(pose);

    // 3.构建边
    int index = 1;
    vector<EdgeProjectXYZRGBDPoseOnly*> edges;
    for (int i = 0; i < pts1.size(); i++)
    {
        EdgeProjectXYZRGBDPoseOnly *edge = new EdgeProjectXYZRGBDPoseOnly(Eigen::Vector3d(pts2[i].x,pts2[i].y,pts2[i].z));
        edge->setId(index);
        edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap*>(pose));
        edge->setMeasurement(Eigen::Vector3d(pts1[i].x,pts1[i].y,pts1[i].z));
        edge->setInformation(Eigen::Matrix3d::Identity()*1e4);
        optimizer.addEdge(edge);
        index++;
        edges.push_back(edge);
    }
	// 开始优化
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout << "优化用时: " << time_used.count() << "ms" << endl;
    cout << "优化后的T: \n: " << endl;
    cout << Eigen::Isometry3d(pose->estimate()).matrix() << endl;
}
    
```

## 主函数

```cpp
int main(int argc, char **argv)
{
    if (argc != 5)
    {
        cout << "提供彩色图与深度图：img1 img2 depth1 depth2 " << endl;
        return 1;
    }
    // 1.特征点匹配
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "一共找到" << matches.size() << "组特征匹配" << endl;

    // 2.建立3D点
    Mat depth_1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);
    Mat depth_2 = imread(argv[4], CV_LOAD_IMAGE_UNCHANGED);
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point3f> pts1, pts2;
    build_3d_points(depth_1, depth_2, K, keypoints_1, keypoints_2, matches, pts1, pts2);

    // 3.SVD求解ICP
    Mat R, t;
    pose_estimation_3d3d(pts1, pts2, R, t);
    cout << "一共有" << pts1.size() << "对3D-3D";
    cout << "ICP分解结果为：" << endl;
    cout << "R: \n"
         << R << endl;
    cout << "t: \n"
         << t << endl;
    // 由于前面的推导是按照pi= Rp′i+t进行的，这里的R,t, 
    // 是第二帧到第一帧的变换，与前面PnP部分是相反的。
    // 所以在输出结果中，我们同时打印了逆变换
    cout << "R_inv: \n " << R.t() << endl; // 求逆
    cout << "t_inv: \n " << -R.t() * t << endl;

    // 4.BA优化
    bundleAdjustment(pts1, pts2, R, t);

    return 0;
}
```

CMakeLsits.txt

```cmake
cmake_minimum_required( VERSION 2.8 )
project( pose_estimation_3d3d )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

# 添加cmake模块以使用g2o
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV 3.1 REQUIRED )
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    "/usr/include/eigen3/"
)

add_executable( pose_estimation_3d3d pose_estimation_3d3d.cpp )
target_link_libraries( pose_estimation_3d3d 
   ${OpenCV_LIBS}
   g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension 
   ${CSPARSE_LIBRARY}
)

```

打印

```c#
max_dist: 94
min_dist: 4
一共找到79组特征匹配
3d-3d pairs: 72
W=  10.871 -1.01948  2.54771
-2.16033  3.85307 -5.77742
 3.94738 -5.79979  9.62203
U=  0.558087  -0.829399 -0.0252034
 -0.428009  -0.313755   0.847565
  0.710878   0.462228   0.530093
V=  0.617887  -0.784771 -0.0484806
 -0.399894  -0.366747   0.839989
  0.676979   0.499631   0.540434
一共有72对3D-3DICP分解结果为：
R: 
[0.9969452349468715, 0.05983347698056557, -0.05020113095482046;
 -0.05932607657705309, 0.9981719679735133, 0.01153858699565957;
 0.05079975545906246, -0.008525103184062521, 0.9986724725659557]
t: 
[0.144159841091821;
 -0.06667849443812729;
 -0.03009747273569774]
R_inv: 
 [0.9969452349468715, -0.05932607657705309, 0.05079975545906246;
 0.05983347698056557, 0.9981719679735133, -0.008525103184062521;
 -0.05020113095482046, 0.01153858699565957, 0.9986724725659557]
t_inv: 
 [-0.1461462958593589;
 0.05767443542067568;
 0.03806388018483625]
iteration= 0     chi2= 18161.146626      time= 4.6935e-05        cumTime= 4.6935e-05     edges= 72       schur= 0
iteration= 1     chi2= 18155.141919      time= 9.614e-06         cumTime= 5.6549e-05     edges= 72       schur= 0
iteration= 2     chi2= 18155.140765      time= 9.938e-06         cumTime= 6.6487e-05     edges= 72       schur= 0
iteration= 3     chi2= 18155.140764      time= 9.516e-06         cumTime= 7.6003e-05     edges= 72       schur= 0
iteration= 4     chi2= 18155.140764      time= 8.688e-06         cumTime= 8.4691e-05     edges= 72       schur= 0
iteration= 5     chi2= 18155.140764      time= 8.358e-06         cumTime= 9.3049e-05     edges= 72       schur= 0
iteration= 6     chi2= 18155.140764      time= 1.0723e-05        cumTime= 0.000103772    edges= 72       schur= 0
iteration= 7     chi2= 18155.140764      time= 1.1911e-05        cumTime= 0.000115683    edges= 72       schur= 0
iteration= 8     chi2= 18155.140764      time= 1.3427e-05        cumTime= 0.00012911     edges= 72       schur= 0
iteration= 9     chi2= 18155.140764      time= 1.207e-05         cumTime= 0.00014118     edges= 72       schur= 0
优化用时: 0.000604832ms
优化后的T: 
: 
  0.996945  0.0598335 -0.0502011    0.14416
-0.0593261   0.998172  0.0115386 -0.0666785
 0.0507998 -0.0085251   0.998672 -0.0300979
         0          0          0          1
```

不使用解析求导

```c#
.......
优化用时: 0.00123263ms
优化后的T: 
: 
   0.996945   0.0598335  -0.0502011     0.14416
 -0.0593261    0.998172   0.0115386  -0.0666785
  0.0507998 -0.00852509    0.998672  -0.0300979
          0           0           0           1
```

**不难发现,使用解析求导(即给出函数`virtual void linearizeOplus()`),比使用数值求导(即不提供该函数)要快的多.**

