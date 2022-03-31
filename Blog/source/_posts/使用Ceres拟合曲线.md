---
title: ch6_使用Ceres拟合曲线
tags: [SLAM实践]
date: {{ date }}
comments: true
mathjax: true
categories: SLAM十四讲
---

{%cq%}

Ceres solver 是谷歌开发的一款用于[非线性](https://so.csdn.net/so/search?q=非线性&spm=1001.2101.3001.7020)优化的库，在谷歌的开源激光雷达 slam 项目 cartographer 中被大量使用。Ceres 官网上的文档非常详细地介绍了其具体使用方法，相比于另外一个在 slam 中被广泛使用的图优化库 G2O，ceres 的文档可谓相当丰富详细。

{%endcq%}

<!-- more -->

下载与编译：

```bash
# 克隆项目
git clone https://github.com/ceres-solver/ceres-solver
# 安装依赖
sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3.1.2 libgflags-dev libgoogle-glog-dev libgtest-dev
# 编译安装
cd ceres-solver
mkdir build
cd build 
cmake ..
make -j4
sudo make install
```

## Ceres使用步骤：

- （1）定义$Cost Function$ 模型

代价函数，也就是寻优的目标式。这个部分需要使用仿函数$（functor）$这一技巧来实现，做法是定义一个$ cost function $的结构体，在结构体内重载（）运算符。

- （2）通过代价函数构建待求解的优化问题

- （3）配置求解器参数并求解问题

## HelloWorld案例

### 1.构建代价函数

本案例中待优化的函数为：$f(x)=10-x$，我们需要寻找最优的$x$值是的函数$f(x)$最小，所以误差项为$10.0-x[0]$。

```cpp
//第一部分：构建代价函数
struct CostFunctor {
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};
```

### 2.构建寻优问题

首先，定义 Problem 类型的变量，然后将构建的代价函数添加到寻优问题中。实例化对象`AutoDiffCostFunction`将创建的代价函数结构体实例作为输入，自动生成其微分并且返回一个`CostFunction` 类型的接口。

```cpp
 // 第二部分：构建寻优问题
    Problem problem;
    CostFunction* cost_function =
    // 使用自动求导入，第一个1是输出维度，即残差的维度，第二个1是输入维度，即待寻优参数x的维度
            new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(cost_function, NULL, &x); //向问题中添加误差项，本问题比较简单，添加一个就行。

```

### 3.配置并运行求解器

为求解这个优化问题，我们需要做一些配置，需要创建一个 Option，配置一下求解器的配置，创建一个 Summary。最后调用 Solve 方法，求解。

```cpp
//第三部分： 配置并运行求解器
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR; //配置增量方程的解法
    options.minimizer_progress_to_stdout = true;//输出到cout
    Solver::Summary summary;//优化信息
    Solve(options, &problem, &summary);//求解
```

### 4.输出结果

```cpp
  std::cout << summary.BriefReport() << "\n";//输出优化的简要信息
    //最终结果
    std::cout << "x : " << initial_x
              << " -> " << x << "\n";
    return 0;
```

输出日志

```bash
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  1.250000e+01    0.00e+00    5.00e+00   0.00e+00   0.00e+00  1.00e+04        0    1.41e-05    4.20e-05
   1  1.249750e-07    1.25e+01    5.00e-04   5.00e+00   1.00e+00  3.00e+04        1    3.10e-05    1.18e-04
   2  1.388518e-16    1.25e-07    1.67e-08   5.00e-04   1.00e+00  9.00e+04        1    3.81e-06    1.31e-04
Ceres Solver Report: Iterations: 3, Initial cost: 1.250000e+01, Final cost: 1.388518e-16, Termination: CONVERGENCE
x : 5 -> 10
```

整体代码：

```cpp
#include <iostream>
#include <ceres/ceres.h>

using namespace std;
using namespace ceres;
//第一部分：构建代价函数
struct CostFunctor {
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};

//主函数
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // 寻优参数x的初始值，为5
    double initial_x = 5.0;
    double x = initial_x;

    // 第二部分：构建寻优问题
    Problem problem;
    CostFunction* cost_function =
            new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor); //使用自动求导
    problem.AddResidualBlock(cost_function, NULL, &x); 
    
    //第三部分： 配置并运行求解器
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR; //配置增量方程的解法
    options.minimizer_progress_to_stdout = true;//输出到cout
    Solver::Summary summary;//优化信息
    Solve(options, &problem, &summary);//求解

    std::cout << summary.BriefReport() << "\n";//输出优化的简要信息
    //最终结果
    std::cout << "x : " << initial_x
              << " -> " << x << "\n";
    return 0;
}
```



## 非线性优化案例

使用$ Ceres $拟合曲线

假设有一条满足以下方程的曲线：
$$
y=exp(ax^2+bx+c)+w
$$
其中$ a, b, c $为曲线的参数，$w $为高斯噪声。我们故意选择了这样一个非线性模型，以使问题不至于太简单。现在，假设我们有$ N $个关于$ x, y$ 的观测数据点，想根据这些数据点求出曲线的参数。那么，可以求解下面的最小二乘问题以估计曲线参数：
$$
\underset{a,b,c}{min}\frac{1}{2}\sum^{N}_{i=1}\parallel y_i-exp(ax^2_i+bx_i+c)\parallel^2
$$
请注意，在这个问题中，待估计的变量是 $a, b, c$，而不是$ x$。我们写一个程序，先根据模型生成 $x, y$ 的真值，然后在真值中添加高斯分布的噪声。随后，使用 $Ceres $从带噪声的数据中拟合参数模型。

```cpp
#include <iostream>
#include <ceres/ceres.h>
#include <opencv2/core/core.hpp>
#include <chrono>
using namespace std;

//第一部分：构建代价函数
struct CURVE_FITTING_COST
{

    //结构体中的自定义构造函数，同时初始化x和y(使用构造函数初始化const值，我们必须使用初始化列表)
    CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}
    // 残差的计算
    template <typename T>
    // const T* const abc模型参数，有3维
    // T* residual      // 残差
    bool operator()(const T *const abc, T *residual) const
    {
        // y-exp(ax^2+bx+c)
        residual[0] = T(_y) - ceres::exp(abc[0]* T(_x)* T(_x) + abc[1] * T(_x) + abc[2]);
        return true;
    }

    const double _x, _y; // x,y数据
};

int main(int argc, char **argv)
{
    double a = 1.0, b = 2.0, c = 1.0; // 真实参数值
    int N = 100;                      // 数据点
    double w_sigma = 1.0;             // 噪声Sigma值
    cv::RNG rng;                      // OpenCV随机数产生器
    double abc[3] = {0, 0, 0};        // abc参数的估计值

    // 创建存放x，y数据的容器
    vector<double> x_data, y_data;
    // 将生成100组的数据给容器
    cout << "产生数据：" << endl;
    for (int i = 0; i < N; i++)
    {
        double x = i / 100.0; // x为0-1之间的100个数
        x_data.push_back(x);
        y_data.push_back(exp(a * x * x + b * x + c) + rng.gaussian(w_sigma));
        cout << x_data[i] << " " << y_data[i] << endl;
    }

     // 第二部分：构建寻优问题
    ceres::Problem problem;
    for (int i = 0; i < N; i++)
    {
        // 向问题中添加误差项
        problem.AddResidualBlock(
            // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
            // 第一个1是输出维度，即残差的维度，第二个3是输入维度，即待寻优参数abc的维度
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>
                        (new CURVE_FITTING_COST(x_data[i], y_data[i])),
            nullptr, // 核函数，这里不使用，为空
            abc);    // 待估计参数
    }

    // 第三部分： 配置并运行求解器
    ceres::Solver::Options options;               // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR; // 增量方程求解方法（QR）
    options.minimizer_progress_to_stdout = true;  // 输出到cout

    //优化信息
    ceres::Solver::Summary summary;                                                                
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();// 计时点t1

    // 开始优化
    ceres::Solve(options,&problem,&summary); //求解!!!!

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();// 计时点t2
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1); // 统计用时
    cout << "总计用时：" << time_used.count() << endl;

    // 输出结果
    cout << summary.BriefReport() << endl;
    cout << "估计的a,b,c的值为：" ;
    for (auto a : abc)
    {
        cout << a << " ";   
    }
    cout << endl;

    return 0;
}

```

CMakeLists.txt

```cmake
cmake_minimum_required( VERSION 2.8 )
project( ceres_curve_fitting )

set( CMAKE_BUILD_TYPE "Release" )
set(CMAKE_CXX_STANDARD 14)

# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

add_executable( curve_fitting ceres_curve_fitting.cpp )
# 与Ceres和OpenCV链接
target_link_libraries( curve_fitting ${CERES_LIBRARIES} ${OpenCV_LIBS} )
```

输出结果：

```bash
产生数据：
0 2.71828
0.01 2.93161
0.02 2.12942
0.03 2.46037
0.04 4.18814
0.05 2.73368
0.06 2.42751
0.07 3.44729
0.08 3.72543
0.09 2.1358
.....
0.96 47.7941
0.97 48.5931
0.98 51.8487
0.99 51.0258
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  1.824887e+04    0.00e+00    1.38e+03   0.00e+00   0.00e+00  1.00e+04        0    1.02e-04    1.95e-04
   1  2.748700e+39   -2.75e+39    1.38e+03   7.67e+01  -1.52e+35  5.00e+03        1    1.02e-04    3.83e-04
   2  2.429783e+39   -2.43e+39    1.38e+03   7.62e+01  -1.35e+35  1.25e+03        1    3.91e-05    4.59e-04
   3  1.213227e+39   -1.21e+39    1.38e+03   7.30e+01  -6.73e+34  1.56e+02        1    3.29e-05    5.27e-04
   4  1.852387e+37   -1.85e+37    1.38e+03   5.56e+01  -1.03e+33  9.77e+00        1    3.22e-05    5.87e-04
   5  6.714689e+31   -6.71e+31    1.38e+03   2.96e+01  -3.85e+27  3.05e-01        1    3.22e-05    6.44e-04
   6  9.500531e+12   -9.50e+12    1.38e+03   9.50e+00  -8.39e+08  4.77e-03        1    3.10e-05    7.00e-04
   7  1.776982e+04    4.79e+02    1.83e+03   2.58e-01   1.18e+00  1.43e-02        1    1.14e-04    8.37e-04
   8  1.599969e+04    1.77e+03    3.45e+03   5.53e-01   1.46e+00  4.29e-02        1    8.11e-05    9.54e-04
   9  1.060557e+04    5.39e+03    7.62e+03   7.33e-01   1.68e+00  1.29e-01        1    1.30e-04    1.11e-03
  10  3.669783e+03    6.94e+03    9.60e+03   5.25e-01   1.39e+00  3.86e-01        1    9.61e-05    1.24e-03
  11  5.397541e+02    3.13e+03    5.00e+03   2.66e-01   1.12e+00  1.16e+00        1    8.20e-05    1.36e-03
  12  1.484444e+02    3.91e+02    1.22e+03   8.46e-02   1.02e+00  3.48e+00        1    8.20e-05    1.48e-03
  13  1.216815e+02    2.68e+01    3.76e+02   4.17e-02   1.01e+00  1.04e+01        1    7.39e-05    1.58e-03
  14  9.290109e+01    2.88e+01    2.42e+02   9.10e-02   1.01e+00  3.13e+01        1    1.57e-04    1.76e-03
  15  6.674330e+01    2.62e+01    1.09e+02   1.33e-01   1.00e+00  9.39e+01        1    1.31e-04    1.93e-03
  16  5.936574e+01    7.38e+00    2.14e+01   1.08e-01   9.94e-01  2.82e+02        1    1.39e-04    2.10e-03
  17  5.653118e+01    2.83e+00    1.36e+01   1.57e-01   9.98e-01  8.45e+02        1    9.11e-05    2.25e-03
  18  5.310764e+01    3.42e+00    8.50e+00   2.81e-01   9.89e-01  2.53e+03        1    7.58e-05    2.35e-03
  19  5.125939e+01    1.85e+00    2.84e+00   2.98e-01   9.90e-01  7.60e+03        1    7.39e-05    2.45e-03
  20  5.097693e+01    2.82e-01    4.34e-01   1.48e-01   9.95e-01  2.28e+04        1    7.41e-05    2.54e-03
  21  5.096854e+01    8.39e-03    3.24e-02   2.87e-02   9.96e-01  6.84e+04        1    7.30e-05    2.64e-03
总计用时：0.00273015
Ceres Solver Report: Iterations: 22, Initial cost: 1.824887e+04, Final cost: 5.096854e+01, Termination: CONVERGENCE
估计的a,b,c的值为：0.891943 2.17039 0.944142 
```

从 Ceres 给出的优化过程中可以看到，整体误差从 18248 左右下降到了 50.9，并且梯度也是越来越小。在迭代 22 次后算法收敛，最后的估计值为：
$$
a=0.891943, b=2.17039, c=0.944142
$$
与真实值比较接近。

![Ceres](Ceres.PNG)

