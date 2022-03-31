---
title: g2o的结构以及BA节点与边的定义
tags: [SLAM实践]
date: {{ date }}
comments: true
mathjax: true
categories: SLAM十四讲

---

{%cq%}

$g2o $提供了许多关于$BA $的节点和边，我们不必自己从头实现所有的计算。在`g2o/types/sba/types_six_dof_expmap.h `中则提供了李代数表达的节点和边。

{%endcq%}

<!-- more -->

# g2o的结构

> 参考博客：[深入理解图优化与g2o：g2o篇](https://www.cnblogs.com/gaoxiang12/p/5304272.html)

源码：https://github.com/RainerKuemmerle/g2o

![image-20220328125016497](image-20220328125016497.png)

- **cmake_modules：**给cmake用来寻找库的文件。我们用g2o时也会用它里头的东西，例如FindG2O.cmake
- **doc：**文档。包括g2o自带的说明书（难度挺大的一个说明文档）。
- **g2o：**最重要的源代码都在这里！
- **script：**在android等其他系统编译用的脚本

https://github.com/RainerKuemmerle/g2o/tree/master/g2o

![image-20220328125308391](image-20220328125308391.png)

- **apps：**一些应用程序。好用的g2o_viewer就在这里。其他还有一些不常用的命令行工具等。
- **core：**核心组件，很重要！基本的顶点、边、图结构的定义，算法的定义，求解器接口的定义在这里。
- **examples：**一些例程，可以参照着这里的东西来写。不过注释不太多。
- **solvers：**求解器的实现。主要来自choldmod, csparse。在使用g2o时要先选择其中一种。
- **stuff：**对用户来讲可有可无的一些工具函数。
- **types：**各种顶点和边，很重要！我们用户在构建图优化问题时，先要想好自己的顶点和边是否已经提供了定义。如果没有，要自己实现。如果有，就用g2o提供的即可。

**就经验而言，solvers给人的感觉是大同小异，而 types 的选取，则是 g2o 用户主要关心的内容。然后 core 下面的内容，我们要争取弄的比较熟悉，才能确保使用中出现错误可以正确地应对。**

g2o最基本的类结构：

![img](606958-20160321233900042-681579456.png)

　先看上半部分。SparseOptimizer 是我们最终要维护的东东。它是一个Optimizable Graph，从而也是一个Hyper Graph。一个 SparseOptimizer 含有很多个顶点 （都继承自 Base Vertex）和很多个边（继承自 BaseUnaryEdge, BaseBinaryEdge或BaseMultiEdge）。这些 Base Vertex 和 Base Edge 都是抽象的基类，而实际用的顶点和边，都是它们的派生类。我们用 SparseOptimizer.addVertex 和 SparseOptimizer.addEdge 向一个图中添加顶点和边，最后调用 SparseOptimizer.optimize 完成优化。

　　在优化之前，需要指定我们用的求解器和迭代算法。从图中下半部分可以看到，一个 SparseOptimizer 拥有一个 Optimization Algorithm，继承自Gauss-Newton, Levernberg-Marquardt, Powell's dogleg 三者之一（我们常用的是GN或LM）。同时，这个 Optimization Algorithm 拥有一个Solver，它含有两个部分。一个是 SparseBlockMatrix ，用于计算稀疏的雅可比和海塞； 一个是用于计算迭代过程中最关键的一步
$$
HΔx=−b
$$
这就需要一个线性方程的求解器。而这个求解器，可以从 PCG, CSparse, Choldmod 三者选一。

**综上所述，在g2o中选择优化方法一共需要三个步骤：**

1. 创建一个线性求解器LinearSolver。从 PCG, CSparse, Choldmod中选，实际则来自 g2o/solvers 文件夹中定义的内容。
2. 创建BlockSolver，并用上面定义的线性求解器初始化。
3. 创建总求解器solver，并从GN/LM/DogLeg 中选一个作为迭代策略，再用上述块求解器BlockSolver初始化。
4. 创建图优化的核心：稀疏优化器（SparseOptimizer）。
5. 定义图的顶点和边，并添加到SparseOptimizer中。
6. 设置优化参数，开始执行优化。

# g2o中定义的顶点和边

$g2o $提供了许多关于$BA $的节点和边，我们不必自己从头实现所有的计算。`g2o/types/sba/types_six_dof_expmap.h `中则提供了李代数表达的节点和边。

因为在不同的应用场景（二维空间，三维空间），有不同的待优化变量（位姿，空间点），还涉及不同的优化类型（李代数位姿、李群位姿），g2o本身内部定义了一些常用的顶点类型

```cpp
VertexSE2 : public BaseVertex<3, SE2>  //2D位姿顶点, (x,y,theta)
VertexSE3 : public BaseVertex<6, Isometry3>  //6d vector (x,y,z,qx,qy,qz) 请注意，我们省略了四元数的 w 部分
VertexPointXY : public BaseVertex<2, Vector2>
VertexPointXYZ : public BaseVertex<3, Vector3>
VertexSBAPointXYZ : public BaseVertex<3, Vector3>

VertexSE3Expmap : public BaseVertex<6, SE3Quat>
VertexCam : public BaseVertex<6, SBACam>
VertexSim3Expmap : public BaseVertex<7, Sim3>
```

## 顶点Vertex

g2o中顶点为**待优化变量**，边为**误差项**，综合之前所提到的非线性优化的应用：

- 求解$PnP$
  - 待优化的变量是：相机的位姿以及所有的特征点的空间位置$P$
  - 误差项：重投影误差 $ξ^{*}=arg \underset{ξ}{min}\frac{1}{2}\sum^n_{i=1}\parallel u_i-\frac{1} {s_i}Kexp(ξ^{\land})P_i\parallel^2_2$

- 求解$ICP$
  - 待优化的变量是：相机的位姿以及所有的特征点的空间位置$P$
  - 误差项：重投影误差$\underset{\xi}{min}=\frac{1}{2}\sum^n_{i=1}\parallel (p_i - exp(\xi^{\land})p^{\prime}_i) \parallel^2_2$

### `VertexSE3Expmap`

VertexSE3Expmap(李代数位姿)

继承于`BaseVertex`这个模板类
需要设置的模板参数：

- 参数`6` ：表示它内部存储的优化变量维度。可以看到这是一个 6 维的李代数。，前三维表示旋转，后三维表示平移
- 参数`SE3Quat` ：表示优化变量的类型。这里使用了 g2o 定义的相机位姿： SE3Quat。这个类内部使用了四元数加位移向量来存储位姿，但同时也支持李代数上的运算，例如对数映射（log 函数）和李代数上增量（update 函数）等操作。

```cpp
class G2O_TYPES_SBA_API VertexSE3Expmap : public BaseVertex<6, SE3Quat> 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        VertexSE3Expmap();
    bool read(std::istream& is);
    bool write(std::ostream& os) const;
    // 顶点重置函数，设定被优化变量的原始值
    void setToOriginImpl() 
    {
        _estimate = SE3Quat(); 
    }
    // 顶点更新函数。主要用于优化过程中增量△x的计算
    void oplusImpl(const number_t* update_) 
    {
          Eigen::Map<const Vector6> update(update_);
          setEstimate(SE3Quat::exp(update) * estimate());
    }
    .....
};
```

#### 参数设置

使用时需要设置的参数，可以参考之前使用$PnP$优化第二个相机的位姿的时设置的参数内容：

```cpp
g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
Eigen::Matrix3d R_mat;
R_mat << 
    R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
pose->setId(0); // 设置Id号
pose->setEstimate(g2o::SE3Quat( // 设置待优化位姿R,t
    R_mat,
    Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))、
));
optimizer.addVertex(pose); // 向一个图中添加顶点
```

### `VertexPointXYZ`

VertexPointXYZ(空间点位置)

```cpp
class G2O_TYPES_SLAM3D_API VertexPointXYZ : public BaseVertex<3, Vector3> 
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexPointXYZ() {}
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
	// 顶点重置函数，设定被优化变量的原始值。
    virtual void setToOriginImpl() { _estimate.fill(0.); }
	// 顶点更新函数。主要用于优化过程中增量△x的计算
    virtual void oplusImpl(const number_t* update_) {
        Eigen::Map<const Vector3> update(update_);
        _estimate += update;
    }
    ....
};
```

#### 参数设置

使用时需要设置的参数，可以参考之前使用$PnP$优化所有特征点的空间位置P时，设置的参数内容

```cpp
// 优化所有特征点的空间位置 P
int index = 1;
for (const Point3f p : points_3d)
{
    g2o::VertexPointXYZ *point = new g2o::VertexPointXYZ();
    point->setId(index++); // 设置Id号
    point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));// 设置待优化空间点3D位置XYZ
    point->setMarginalized(true); // 是否边缘化（以便稀疏化求解）g2o中必须设置 marg 参见第十讲内容
    optimizer.addVertex(point); // 向一个图中添加顶点
}
```

## 边Edge

BaseUnaryEdge，BaseBinaryEdge，BaseMultiEdge 分别表示一元边，两元边，多元边。一元边可以理解为一条边只连接一个顶点，两元边理解为一条边连接两个顶点，也就是我们常见的边啦，多元边理解为一条边可以连接多个（3个以上）顶点

![](5c7df32d30e9d.png)

### EdgeProjectXYZ2UV

EdgeProjectXYZ2UV(重投影误差)

```cpp
class G2O_TYPES_SBA_API EdgeProjectXYZ2UV : public BaseBinaryEdge<2, Vector2, VertexPointXYZ, VertexSE3Expmap> 
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    //1. 默认初始化
    EdgeProjectXYZ2UV();
    bool read(std::istream& is);
    bool write(std::ostream& os) const;
    //2. 计算误差
    void computeError(); // 使用当前顶点的值计算的测量值与真实的测量值之间的误差
    //3. 线性增量函数，也就是雅克比矩阵J的计算方法
    virtual void linearizeOplus(); // 在当前顶点的值下，该误差对优化变量的偏导数，也就是我们说的Jacobian

 public:
  //相机参数
  CameraParameters* _cam;  // TODO make protected member?
};
```

这是个二元边。第1个参数2是说观测值是2维的，也就是图像像素坐标x,y的差值，即$ z − h(ξ, P )$，对应观测值的类型是Vector2D，两个顶点也就是优化变量分别是三维点 VertexPointXYZ，和李群位姿VertexSE3Expmap

#### computeError()

对应的`computeError() `的实现：

```cpp
void EdgeProjectXYZ2UV::computeError() {
  const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
  const VertexPointXYZ* v2 = static_cast<const VertexPointXYZ*>(_vertices[0]);
  const CameraParameters* cam =
      static_cast<const CameraParameters*>(parameter(0));
  // 误差 = 观测 - 投影
  _error = measurement() - cam->cam_map(v1->estimate().map(v2->estimate()));
}
```

 `_vertices[0] `对应的是 VertexPointXYZ 类型的顶点，也就是三维点，`_vertices[1] `对应的是VertexSE3Expmap 类型的顶点，也就是位姿pose。因此前面 1 对应的就应该是 pose，0对应的应该就是三维点。

cam_map 函数功能是把相机坐标系下三维点（输入）用内参转换为图像坐标（输出），具体代码如下所示:

```cpp
Vector2 CameraParameters::cam_map(const Vector3 &trans_xyz) const 
{
    Vector2 proj = project(trans_xyz);
    Vector2 res;
    res[0] = proj[0] * focal_length + principle_point[0];
    res[1] = proj[1] * focal_length + principle_point[1];
    return res;
}
```

.map的功能是把世界坐标系下三维点变换到相机坐标系

```cpp
Vector3 map(const Vector3& xyz) const { return _r * xyz + _t; }
```

因此下面这个代码

```cpp
v1->estimate().map(v2->estimate())
```

就是用V1估计的pose把V2代表的三维点，变换到相机坐标系下。

#### linearizeOplus()

对应的`linearizeOplus() `的实现：

```cpp
void EdgeProjectXYZ2UV::linearizeOplus() {
  VertexSE3Expmap* vj = static_cast<VertexSE3Expmap*>(_vertices[1]);
  SE3Quat T(vj->estimate());
  VertexPointXYZ* vi = static_cast<VertexPointXYZ*>(_vertices[0]);
  Vector3 xyz = vi->estimate();
  Vector3 xyz_trans = T.map(xyz);

  number_t x = xyz_trans[0];
  number_t y = xyz_trans[1];
  number_t z = xyz_trans[2];
  number_t z_2 = z * z;

  const CameraParameters* cam =
      static_cast<const CameraParameters*>(parameter(0));

  Eigen::Matrix<number_t, 2, 3, Eigen::ColMajor> tmp;
  tmp(0, 0) = cam->focal_length;
  tmp(0, 1) = 0;
  tmp(0, 2) = -x / z * cam->focal_length;

  tmp(1, 0) = 0;
  tmp(1, 1) = cam->focal_length;
  tmp(1, 2) = -y / z * cam->focal_length;

  _jacobianOplusXi = -1. / z * tmp * T.rotation().toRotationMatrix();

  _jacobianOplusXj(0, 0) = x * y / z_2 * cam->focal_length;
  _jacobianOplusXj(0, 1) = -(1 + (x * x / z_2)) * cam->focal_length;
  _jacobianOplusXj(0, 2) = y / z * cam->focal_length;
  _jacobianOplusXj(0, 3) = -1. / z * cam->focal_length;
  _jacobianOplusXj(0, 4) = 0;
  _jacobianOplusXj(0, 5) = x / z_2 * cam->focal_length;

  _jacobianOplusXj(1, 0) = (1 + y * y / z_2) * cam->focal_length;
  _jacobianOplusXj(1, 1) = -x * y / z_2 * cam->focal_length;
  _jacobianOplusXj(1, 2) = -x / z * cam->focal_length;
  _jacobianOplusXj(1, 3) = 0;
  _jacobianOplusXj(1, 4) = -1. / z * cam->focal_length;
  _jacobianOplusXj(1, 5) = y / z_2 * cam->focal_length;
}
```

成员变量`_jacobianOplusXi`是**误差到空间点的导数**，`_jacobianOplusXj`是误**差到相机位姿的导数**，以李代数的左乘扰动表达。稍有差别的是，g2o 的相机里用 $f$ 统一描述 $f_x , f_y$ ，并且李代数定义顺序不同（g2o 是旋转在前，平移在后；我们是平移在前，旋转在后），所以矩阵前三列和后三列与我们的定义是颠倒的。此外都是一致的。

李代数定义的$J$：（）
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
&-f_y-\frac{f_yY^{′2}}{Z^{′2}}&\frac{f_yX^′Y^′}{Z^{′}}&-\frac{f_yX^′}{Z^{′}}
\end{bmatrix}\\&=
J
\end{align}
$$
而g2o中的$J$定义的方式如下：（旋转在前，平移在后）
$$
-
\begin{bmatrix}
-\frac{f_xX^′Y^′}{Z^{′2}}&f_x+\frac{f_xX^2}{Z^{′2}}&-\frac{f_xY^′}{Z^{′}}
&\frac{f_x}{Z^′}&0&-\frac{f_xX^′}{Z^{′2}}
\\
-f_y-\frac{f_yY^{′2}}{Z^{′2}}&\frac{f_yX^′Y^′}{Z^{′}}&-\frac{f_yX^′}{Z^{′}}
&0&\frac{f_y}{Z^′}&-\frac{f_yY^′}{Z^{′2}}
\end{bmatrix}
$$
值得一提的是，我们亦可自己实现相机位姿节点，并使用` Sophus::SE3 `来表达位姿，提供类似的求导过程。然而，既然 g2o 已经提供了这样的类，在没有额外要求的情况下，自己重新实现就没有必要了。

使用时需要设置的参数，以$PnP$中重投影误差为例：

```cpp
// 2.设置边(每个 3D 点在第二个相机中的投影)
index = 1;
for (const Point2f p : points_2d)
{
    g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
    edge->setId(index); //定义边的编号（决定了在H矩阵中的位置）
    edge->setVertex(0, dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(index)));
    edge->setVertex(1, pose); // 定义顶点
    edge->setMeasurement(Eigen::Vector2d(p.x, p.y)); // 定义观测值
    edge->setParameterId(0, 0);
    edge->setInformation(Eigen::Matrix2d::Identity()) ; // 定义协方差矩阵的逆
    optimizer.addEdge(edge);
    index++;
}
```

# 自定义顶点类型

一般情况下，如果我们需要用的顶点已经提供，则直接使用即可，但是有时候我们需要的顶点类型这里面没有，就得自己定义了。重新定义顶点一般需要考虑重写如下函数：

```cpp
// 顶点重置函数，设定被优化变量的原始值。
virtual void setToOriginImpl();
// 顶点更新函数。主要用于优化过程中增量△x的计算
virtual void oplusImpl(const number_t* update);
// 读盘、存盘函数，一般情况下不需要进行读/写操作的话，仅仅声明一下就可以
virtual bool read(std::istream& is);
virtual bool write(std::ostream& os) const;
```

`virtual void setToOriginImpl();`顶点重置函数，设定被优化变量的原始值。

`virtual void oplusImpl(const number_t* update);`顶点更新函数。主要用于优化过程中增量△x的计算。我们根据增量方程计算出增量之后，就是通过这个函数对估计值进行调整的。

自定义顶点的格式：

```cpp
class myVertex: public g2::BaseVertex<Dim, Type>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    myVertex(){}

    virtual void read(std::istream& is) {}
    virtual void write(std::ostream& os) const {}

    virtual void setOriginImpl()
    {
    _estimate = Type();
    }
    virtual void oplusImpl(const double* update) override
    {
    _estimate += /*update*/;
    }
}
```

# 自定义边的类型

在使用BA求解ICP的实践中,使用的是3D-3D的边,由于g2o本身没有提供3D-3D的边,因此需自定义一个3D-3D的边:

自定义边格式

```cpp
class myEdge : public g2o::BaseUnaryEdge<n,Dim, Type>
{
public:
    // 结构体包含eigen成员必须进行宏定义 EIGEN_MAKE_ALIGNED_OPERATOR_NEW, 保证内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // 构造函数
    myEdge(){}

    // 再添加边之前需要设定好该边连接的节点的索引以及观测值和信息矩阵，
    // 这个函数则是使用该边连接的节点和观测值来计算误差
    virtual void computeError(){}

    // 计算雅克比矩阵，这个函数是可选的，如果给出了则进行解析求导，不给则进行数值求导
    virtual void linearizeOplus(){}
    // 存盘和读盘：留空
    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {}

protected:
    // 属性
}
```

以构建3D-3D的边为例:

主要考虑两个函数的实现

- `virtual void computeError()`

   在添加边之前需要设定好该边连接的节点的索引以及观测值和信息矩阵，这个函数则是使用该边连接的节点和观测值来计算误差

```cpp
virtual void computeError()
{
    const g2o::VertexSE3Expmap *pose = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
    // measurement is p, point is p'
    _error = _measurement - pose->estimate().map(_point); //.map的功能是把世界坐标系下三维点变换到相机坐标系
}

```

这里误差的定义方式,是考虑相邻两帧图像之间的重投影误差,比如我们通过特征点匹配,确定了两个匹配好的特征点,$p_i$和$p_i^{\prime}$他们的关系为:
$$
p_i = Rp_i^{\prime}+t
$$
由于一些噪声,我们发现依据所求得的相机位姿$R,t$并没有达到理想的准确度,即$Rp_i^{\prime}+t \neq p_i$,他们的差值我们就定义为误差项:
$$
e_i = p_i - (Rp_i^{\prime}+t)
$$
因此,我们把估计的$p_i^{\prime}$的世界坐标系下三维点转换成相机坐标系,并与$p_i$做差

- `virtual void linearizeOplus()`

计算雅克比矩阵，这个函数是可选的，如果给出了则进行解析求导，不给则进行数值求导,但是就优化效率而言,前者计算速度是更快的

```cpp
// 计算雅克比矩阵，这个函数是可选的，如果给出了则进行解析求导，不给则进行数值求导
    virtual void linearizeOplus()
    {
        // VertexSE3Expmap，这个表示李代数的位姿；
        g2o::VertexSE3Expmap *pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat T(pose->estimate());          // 顶点的当前估计
        Eigen::Vector3d xyz_trans = T.map(_point); // .map的功能是把世界坐标系下三维点变换到相机坐标系
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        //_jacobianOplusXi是误差关于世界坐标系下坐标点的偏导
        _jacobianOplusXi(0, 0) = 0;
        _jacobianOplusXi(0, 1) = -z;
        _jacobianOplusXi(0, 2) = y;
        _jacobianOplusXi(0, 3) = -1;
        _jacobianOplusXi(0, 4) = 0;
        _jacobianOplusXi(0, 5) = 0;

        _jacobianOplusXi(1, 0) = z;
        _jacobianOplusXi(1, 1) = 0;
        _jacobianOplusXi(1, 2) = -x;
        _jacobianOplusXi(1, 3) = 0;
        _jacobianOplusXi(1, 4) = -1;
        _jacobianOplusXi(1, 5) = 0;

        _jacobianOplusXi(2, 0) = -y;
        _jacobianOplusXi(2, 1) = x;
        _jacobianOplusXi(2, 2) = 0;
        _jacobianOplusXi(2, 3) = 0;
        _jacobianOplusXi(2, 4) = 0;
        _jacobianOplusXi(2, 5) = -1;
    }

    // 存盘和读盘：留空
    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {}

protected:
    Eigen::Vector3d _point;
};
```

目前代码待理解......
