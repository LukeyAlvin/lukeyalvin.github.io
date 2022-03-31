---
title: ch6_使用g2o拟合曲线
tags: [SLAM实践]
date: {{ date }}
comments: true
mathjax: true
categories: SLAM十四讲
---

{%cq%}

g2o（General Graphic Optimization，G2O）。它是一个基于图优化的库。图优化，是把优化问题表现成图（Graph）的一种方式。这里的图是图论意义上的图。一个图由若干个**顶点（Vertex）**，以及连接着这些节点的**边（Edge）**组成。进而，用**顶点表示优化变量**，用**边表示误差项**。最基本的图优化，是用图模型来表达一个非线性最小二乘的优化问题。

{%endcq%}

<!-- more -->

编译与安装

```bash
# 克隆项目
git clone https://github.com/RainerKuemmerle/g2o
# 安装依赖
sudo apt-get install libqt4-dev qt4-qmake libqglviewer-dev libsuitesparse-dev libcxsparse3.1.2 libcholmod-dev
# 编译安装
cd g2o
mkdir build
cd build 
cmake ..
make -j4
sudo make install
```

## 使用G2O的步骤：

![img](1855613-20200414153348238-1860777008.png)

### 1、创建线性求解器

这里的线性求解器`LinearSolver`创建的是`LinearSolverCSparse`：

- **LinearSolverCholmod ：**使用sparse cholesky分解法。继承自LinearSolverCCS
- **LinearSolverCSparse：**使用CSparse法。继承自LinearSolverCCS
- **LinearSolverDense ：**使用dense cholesky分解法。继承自LinearSolver
- **LinearSolverEigen：** 依赖项只有eigen，使用eigen中sparse Cholesky 求解，因此编译好后可以方便的在其他地方使用，性能和CSparse差不多。继承自LinearSolver
- **LinearSolverPCG ：**使用preconditioned conjugate gradient 法，继承自LinearSolver

### 2、矩阵块求解器

BlockSolver 内部包含 LinearSolver，用上面我们定义的线性求解器LinearSolver来初始化。 

```cpp
Block *solver_ptr = new Block(unique_ptr<Block::LinearSolverType>(linearSolver));
```

### 3、创建总求解器solver。

```cpp
// 梯度下降方法，从GN(高斯牛顿), LM（列文伯格）, DogLeg 中选
g2o::OptimizationAlgorithmGaussNewton *solver_gn = new g2o::OptimizationAlgorithmGaussNewton(unique_ptr<Block>(solver_ptr));
g2o::OptimizationAlgorithmLevenberg *solver_lm = new g2o::OptimizationAlgorithmLevenberg(unique_ptr<Block>(solver_ptr));
g2o::OptimizationAlgorithmDogleg *solver_dog = new g2o::OptimizationAlgorithmDogleg(unique_ptr<Block>(solver_ptr));
```

### 4、创建稀疏优化器

```cpp
g2o::SparseOptimizer optimizer;    // 创建稀疏优化器
optimizer.setAlgorithm(solver_lm); // 用前面定义好的求解器作为求解方法：（使用LM方法）
optimizer.setVerbose(true);        // setVerbose是设置优化过程输出信息用
```

### 5、定义图的顶点和边

定义图的顶点和边。并添加到SparseOptimizer中。

#### 顶点的定义

节点的定义比较简单，只需要继承 G2O 的基础节点类型，并且给出几个关键虚函数的实现即可：

- `setToOriginImpl()`：重置节点的实现，通常情况下各个参数置 0 即可
- `oplusImpl()`：更新节点方法的实现，这里由于我们使用的节点和更新量都是 `Eigen::Vector3d`，直接相加即可
- `read(std::istream &in); & write(std::ostream &out);`：读盘和写盘，还没有深入了解，不过大部分时间不需要

```cpp
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    // 结构体包含eigen成员必须进行宏定义 EIGEN_MAKE_ALIGNED_OPERATOR_NEW, 保证内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl() // 顶点重置函数
    {
        _estimate << 0, 0, 0;
    }

    virtual void oplusImpl(const double *update) // 顶点更新函数
    {
        _estimate += Eigen::Vector3d(update);
    }

    // 存盘和读盘：留空
    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {} // const不声明该类就是一个抽象类，不能进行实例化
};
```

#### 边的定义

边的定义需要继承 G2O 中的一个基础边类型，并给出几个关键函数的实现：

- `computeError()`： 在添加边之前需要设定好该边连接的节点的索引以及观测值和信息矩阵，这个函数则是使用该边连接的节点和观测值来计算误差。最后要更新变量 `_error`
- `linearizeOplus()`：计算雅克比矩阵，这个函数是可选的，如果给出了则进行解析求导，不给则进行数值求导
- `read(std::istream &in); & write(std::ostream &out);`：读盘和写盘

```cpp
// 误差模型 模板参数：观测值维度，类型，连接顶点类型   (边为误差项)
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 构造函数
    CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {} // 初始化列表
    // 计算曲线模型误差
    void computeError()
    {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error(0, 0) = _measurement - std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));
    }
    // 存盘和读盘：留空
    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {} // const不声明该类就是一个抽象类，不能进行实例化

public:
    double _x; // x 值， y 值为 _measurement
};
```



### 6、设置优化参数，开始执行优化

设置SparseOptimizer的初始化、迭代次数、保存结果等

```cpp
optimizer.initializeOptimization(); // 设置优化初始值
optimizer.optimize(100);            // 设置优化次数
```



## 非线性拟合案例

假设有一条满足以下方程的曲线：
$$
y=exp(ax^2+bx+c)+w
$$
其中$ a, b, c $为曲线的参数，$w $为高斯噪声。我们故意选择了这样一个非线性模型，以使问题不至于太简单。现在，假设我们有$ N $个关于$ x, y$ 的观测数据点，想根据这些数据点求出曲线的参数。那么，可以求解下面的最小二乘问题以估计曲线参数：
$$
\underset{a,b,c}{min}\frac{1}{2}\sum^{N}_{i=1}\parallel y_i-exp(ax^2_i+bx_i+c)\parallel^2
$$
节点为优化变量，边为误差项：

![image-20220326161220343](image-20220326161220343.png)

在曲线拟合问题中，整个问题只有一个顶点：曲线模型的参数$ a, b, c$；而每个带噪声的数据点，构成了一个个误差项，也就是图优化的边。

```cpp
#include <iostream>
#include <chrono>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>     // 顶点
#include <g2o/core/base_unary_edge.h> // 边
#include <g2o/core/block_solver.h>
#include <opencv2/core/core.hpp>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h> // GN
#include <g2o/core/optimization_algorithm_levenberg.h>    // LM
#include <g2o/core/optimization_algorithm_dogleg.h>       // DogLeg

using namespace std;

// 曲线模型的顶点，模板参数：优化变量维度和数据类型   (节点为优化变量)
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    // 结构体包含eigen成员必须进行宏定义 EIGEN_MAKE_ALIGNED_OPERATOR_NEW, 保证内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl() // 顶点重置函数
    {
        _estimate << 0, 0, 0;
    }

    virtual void oplusImpl(const double *update) // 顶点更新函数
    {
        _estimate += Eigen::Vector3d(update);
    }

    // 存盘和读盘：留空
    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {} // const不声明该类就是一个抽象类，不能进行实例化
};

// 误差模型 模板参数：观测值维度，类型，连接顶点类型   (边为误差项)
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 构造函数
    CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {} // 初始化列表
    // 计算曲线模型误差
    void computeError()
    {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error(0, 0) = _measurement - std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));
    }
    // 存盘和读盘：留空
    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {} // const不声明该类就是一个抽象类，不能进行实例化

public:
    double _x; // x 值， y 值为 _measurement
};

//主函数
int main(int argc, char **argv)
{
    double a = 1.0, b = 2.0, c = 1.0; // 真实参数值
    int N = 100;                      // 数据点
    double w_sigma = 1.0;             // 噪声Sigma值
    cv::RNG rng;                      // OpenCV随机数产生器
    double abc[3] = {0, 0, 0};        // abc参数的估计值

    vector<double> x_data, y_data;
    cout << "生成数据：" << endl;
    for (int i = 0; i < N; i++)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(a * x * x + b * x + c) + rng.gaussian(w_sigma));
        cout << x_data[i] << " " << y_data[i] << endl;
    }

    // 构建图优化，先设定g2o
    // 每个误差项优化变量维度为3，误差值维度为1
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> Block;
    // 实例化线性方程求解器
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    // 实例化矩阵块求解器
    Block *solver_ptr = new Block(unique_ptr<Block::LinearSolverType>(linearSolver));

    // 梯度下降方法，从GN(高斯牛顿), LM（列文伯格）, DogLeg 中选
    g2o::OptimizationAlgorithmGaussNewton *solver_gn = new g2o::OptimizationAlgorithmGaussNewton(unique_ptr<Block>(solver_ptr));
    g2o::OptimizationAlgorithmLevenberg *solver_lm = new g2o::OptimizationAlgorithmLevenberg(unique_ptr<Block>(solver_ptr));
    g2o::OptimizationAlgorithmDogleg *solver_dog = new g2o::OptimizationAlgorithmDogleg(unique_ptr<Block>(solver_ptr));

    g2o::SparseOptimizer optimizer;    // 图模型
    optimizer.setAlgorithm(solver_lm); //设置求解器（使用LM方法）
    optimizer.setVerbose(true);        // 打开调试输出

    // 往图中增加顶点
    CurveFittingVertex *v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(0, 0, 0)); // 设置优化初始值
    v->setId(0);                              // 设置顶点ID
    optimizer.addVertex(v);                   // 向稀疏优化器添加顶点

    // 往图中增加边
    for (int i = 0; i < N; i++)
    {
        CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
        edge->setId(i);
        edge->setVertex(0, v);           // 设置连接的顶点
        edge->setMeasurement(y_data[i]); //观测数值
        // 信息矩阵：协方差矩阵的逆
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma));
        optimizer.addEdge(edge); // 向稀疏优化器添加边
    }

    // 执行优化
    cout << " 开始优化：" << endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now(); // 计时工具

    optimizer.initializeOptimization(); // 设置优化初始值
    optimizer.optimize(100);            // 设置优化次数

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "求解优化共使用了" << time_used.count() << "秒" << endl;

    // 输出优化值
    Eigen::Vector3d abc_estimate = v->estimate(); // 执行优化
    cout << "估计模型为：" << abc_estimate.transpose() << endl;

    return 0;
}

```

CMakeList.txt

```cmake
cmake_minimum_required( VERSION 2.8 )
project( g2o_curve_fitting )

set( CMAKE_BUILD_TYPE "Release" )
set(CMAKE_CXX_STANDARD 14)

# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找G2O
find_package( G2O REQUIRED )
include_directories( 
    ${G2O_INCLUDE_DIRS}
    "/usr/include/eigen3"
)

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

add_executable( curve_fitting g2o_curve_fitting.cpp )
# 与G2O和OpenCV链接
target_link_libraries( curve_fitting 
    ${OpenCV_LIBS}
    g2o_core g2o_stuff
)
```

输出

```basic
生成数据：
0 2.71828
0.01 2.93161
0.02 2.12942
.......
0.96 47.7941
0.97 48.5931
0.98 51.8487
0.99 51.0258
 开始优化：
iteration= 0     chi2= 30373.727656      time= 7.3362e-05        cumTime= 7.3362e-05     edges= 100      schur= 0        lambda= 699.050482      levenbergIter= 7
iteration= 1     chi2= 13336.948287      time= 4.4188e-05        cumTime= 0.00011755     edges= 100      schur= 0        lambda= 1864.134619     levenbergIter= 3
iteration= 2     chi2= 6946.262238       time= 3.8416e-05        cumTime= 0.000155966    edges= 100      schur= 0        lambda= 1242.756412     levenbergIter= 1
iteration= 3     chi2= 271.023143        time= 3.6174e-05        cumTime= 0.00019214     edges= 100      schur= 0        lambda= 414.252137      levenbergIter= 1
iteration= 4     chi2= 118.903888        time= 3.7968e-05        cumTime= 0.000230108    edges= 100      schur= 0        lambda= 138.084046      levenbergIter= 1
iteration= 5     chi2= 113.568661        time= 3.7864e-05        cumTime= 0.000267972    edges= 100      schur= 0        lambda= 46.028015       levenbergIter= 1
iteration= 6     chi2= 107.476468        time= 3.5648e-05        cumTime= 0.00030362     edges= 100      schur= 0        lambda= 15.342672       levenbergIter= 1
iteration= 7     chi2= 103.014521        time= 4.424e-05         cumTime= 0.00034786     edges= 100      schur= 0        lambda= 5.114224        levenbergIter= 1
iteration= 8     chi2= 101.988349        time= 3.7035e-05        cumTime= 0.000384895    edges= 100      schur= 0        lambda= 1.704741        levenbergIter= 1
iteration= 9     chi2= 101.937388        time= 3.6393e-05        cumTime= 0.000421288    edges= 100      schur= 0        lambda= 0.568247        levenbergIter= 1
iteration= 10    chi2= 101.937021        time= 3.0743e-05        cumTime= 0.000452031    edges= 100      schur= 0        lambda= 0.378831        levenbergIter= 1
iteration= 11    chi2= 101.937020        time= 3.2616e-05        cumTime= 0.000484647    edges= 100      schur= 0        lambda= 0.252554        levenbergIter= 1
iteration= 12    chi2= 101.937020        time= 3.1629e-05        cumTime= 0.000516276    edges= 100      schur= 0        lambda= 0.168370        levenbergIter= 1
iteration= 13    chi2= 101.937020        time= 3.2006e-05        cumTime= 0.000548282    edges= 100      schur= 0        lambda= 0.112246        levenbergIter= 1
iteration= 14    chi2= 101.937020        time= 3.124e-05         cumTime= 0.000579522    edges= 100      schur= 0        lambda= 0.074831        levenbergIter= 1
iteration= 15    chi2= 101.937020        time= 5.3317e-05        cumTime= 0.000632839    edges= 100      schur= 0        lambda= 13391510.122618         levenbergIter= 8
iteration= 16    chi2= 101.937020        time= 3.736e-05         cumTime= 0.000670199    edges= 100      schur= 0        lambda= 857056647.847525        levenbergIter= 3
求解优化共使用了0.00163436秒
估计模型为：0.890912   2.1719 0.943629
```

