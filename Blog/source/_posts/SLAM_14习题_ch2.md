---
title: SLAM_14习题_ch2
tags: [SLAM十四讲]
date: {{ date }}
mathjax: true
comments: true
categories: SLAM十四讲


---

{%cq%}
本次习题，你需要使⽤ Eigen 库，编写程序，求解⼀个线性⽅程组。为此，你需要先
了解⼀些有关线性⽅程组数值解法的原理。

{%endcq%}

<!-- more -->

Eigen（http://eigen.tuxfamily.org）是常⽤的 C++ 矩阵运算库，具有很⾼的运算效率。⼤部分
需要在 C++ 中使⽤矩阵运算的库，都会选⽤ Eigen 作为基本代数库，例如 Google Tensorflow，Google
Ceres，GTSAM 等。本次习题，你需要使⽤ Eigen 库，编写程序，求解⼀个线性⽅程组。为此，你需要先
了解⼀些有关线性⽅程组数值解法的原理。
设线性⽅程 Ax = b，在 A 为⽅阵的前提下，请回答以下问题：

提⽰：你可能需要参考相关的数学书籍或⽂章。请善⽤搜索引擎。Eigen 固定⼤⼩矩阵最⼤⽀持到 50，
所以你会⽤到动态⼤⼩的矩阵。

>  1.在什么条件下，x 有解且唯⼀？

定理  n 元线性方程组$ Ax = b$
（i） **无解**的充分必要条件是$ R（A）＜R（A，b）$；
（ii） **有惟一解**的充分必要条件是$ R（A）= R（A，b）= n$；
（iii） **有无限多解**的充分必要条件是 $R（A）= R（A，b）＜n.$

>  2.⾼斯消元法的原理是什么？

高斯消元法主要是通过用**初等行变换**将增广矩阵化为行阶梯阵，然后通过回带求解线性方程组的解。

![img](/home/alvin/Documents/lukeyalvin.github.io/Blog/source/_posts/img/v2-43c79654d421239b9a6e262b9d91acbe_b.jpg)

>  3.QR 分解的原理是什么？

正交阵的定义:

- 如果 n 阶矩阵 A 满足$A ^T A = E （即 A^{-1} = A^T ）$，那么称 A 为正交矩阵，简称**正交阵**.

QR分解:

- 将矩阵转化成正交矩阵和上三角矩阵的乘积，对应的分解公式是$A=Q*R$。(其中 Q 是正交阵， R 是 upper triangle 矩阵.)

```c++
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
// #define MATRIX_SIZE 100
using namespace std;
using namespace Eigen;
int main()
{
    // Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> matrix_100;
    // matrix_100 = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    // cout << "matrix_100:" << endl << matrix_100 << endl;

    //将矩阵分解为QR
    Matrix4d A;
    A << 2, -1, -1, 1,
        1, 1, -2, 1,
        4, -6, 2, -2,
        3, 6, -9, 7;
    //使用QR分解
    HouseholderQR<MatrixXd> marix_dynamic;
    marix_dynamic.compute(A);

    MatrixXd R = marix_dynamic.matrixQR().triangularView<Eigen::Upper>();

    MatrixXd Q = marix_dynamic.householderQ();

    cout << "A = " << endl << A << endl << endl;
    cout << "R = " << endl << R << endl << endl;
    cout << "Q = " << endl << Q << endl << endl;


    //求解方程AX = B的方法
    Vector4d b(2,4,4,9);
    Vector4d x_qr = A.colPivHouseholderQr().solve(b);  // No pivoting.  
    cout << "x_qr = " << endl << x_qr << endl << endl;
    
    return 0;
}

```



>  4.Cholesky 分解的原理是什么？

将一个正定的埃尔米特矩阵分解成一个下三角矩阵与其共轭转置之乘积。这种分解方式在提高代数运算效率、蒙特卡罗方法等场合中十分有用。实数矩阵的科列斯基分解由安德烈·路易·科列斯基最先发明。实际应用中，科列斯基分解在求解线性方程组中的效率约两倍于LU分解。

关于矩阵分解在Eigen中的使用:https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html



>  5编程实现 A 为 100 × 100 随机矩阵时，⽤ QR 和 Cholesky 分解求 x 的程序。你可以参考本次课
> ⽤到的 useEigen 例程。



