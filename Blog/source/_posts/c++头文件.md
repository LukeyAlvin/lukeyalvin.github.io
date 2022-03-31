---
title: c++头文件
tags: [c++]
date: {{ date }}
comments: true
mathjax: true是
categories: c++

---

{%cq%}
   面向对象的基本特点:一切东西都是对象;程序就是一堆互相传递消息告诉对方需要做什么的对象;没一个对象都有自己的内存,内存里面还是对象;任何对象都有一种类型;一个特定类型的所有对象可以接收相同的消息(反过来说,所有能接收相同消息的对象可以认为是同一个类型;

{%endcq%}

<!-- more -->

# 面向对象

面向对象的基本特点:

> 一切东西都是对象
>
> 程序就是一堆互相传递消息告诉对方需要做什么的对象
>
> 没一个对象都有自己的内存,内存里面还是对象
>
> 任何对象都有一种类型
>
> 一个特定类型的所有对象可以接收相同的消息(反过来说,所有能接收相同消息的对象可以认为是同一个类型)

每个对象都有一个接口:

> 接口是接受消息的一种方式
>
> 接口被定义在这个对象所属的类别里

隐藏的实现(The Hidden Implementation):

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-11_16-47.png" width="60%"/>

# 头文件

一般来说,我们每一个类都应该新建一个`.cpp`文件和一个与之对应的`.h`文件;

`.h`文件用来**声明(declaration)** 方法和变量,旨在告诉你这个类的大体结构是什么,能干嘛?有什么变量?

> 我们称`.h`文件里的class为构造函数
>
> > 作用:相当于接口
> >
> > - The header is a **contract** between you and the user of your code
> > - The compile **enforces the contract** by requiring you to declare all structures and function before they are used.

`.cpp`文件则是核心代码部分,**定义(definition)**某些变量或者方法以及逻辑实现.而这些方法和变量都是`.h`文件所**声明**的.

在`.cpp`文件使用的时候,我们一般使用`#include`来引入`.h`文件中**声明**的方法或者变量,`#include`的含义如下图所示;

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-11_17-06.png" width="60%"/>

# 在Unix查看C++的编译过程

我们新建两个文件,`a.h`和`a.cpp`,如下

```c++
//a.h
void f();
int global;
```

```c++
//a.cpp
#include "a.h"
int main(){}
```

然后使用命令`g++ a.cpp --save-temps `编译,表示保留编译过程中的文件

```sh
-rw-rw-r-- 1 alvin alvin    41 Nov 11 19:49 a.cpp
-rw-rw-r-- 1 alvin alvin    22 Nov 11 19:39 a.h
-rw-rw-r-- 1 alvin alvin   198 Nov 11 19:49 a.ii  #编译预处理指令结束后的结果
-rw-rw-r-- 1 alvin alvin  1400 Nov 11 19:49 a.o  # 目标代码
-rwxrwxr-x 1 alvin alvin 16496 Nov 11 19:49 a.out* # 最终可以执行的代码
-rw-rw-r-- 1 alvin alvin   654 Nov 11 19:49 a.s # 汇编代码
```

查看文件`a.ii`,可以查看到头文件先被抄进来,与`cpp`文件组合在一起,被编译器去执行,这也就印证了之前的那一张图.

```sh
# 1 "a.cpp"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "/usr/include/stdc-predef.h" 1 3 4
# 1 "<command-line>" 2
# 1 "a.cpp"
# 1 "a.h" 1
void f();
int global;
# 2 "a.cpp" 2
int main()
{
 return 0;
}     
```

# 头文件的易错点

## 易错点一:声明与定义

### 案例

在之前的`a.h`中的代码,一个函数f还有一个变量`global`,但是它们并全非**声明**,变量`global`是一种**定义**!这么做是错误的,为什么?举个例子:

我们新建一个文件`b.cpp`

```c++
//b.cpp
#include "a.h"
void f(){}
```

现在需要将`a.cpp`和`b.cpp`组合在一起,执行命令`g++ a.cpp b.cpp --save-temps`

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-11_20-17.png" width="50%"/>

`multiple definition of global`?显然`a.o` `b.o`重复定义了global,查看临时文件,可以看出编译已经完成(这是因为编译器是分开编译`a.cpp`和`b.cpp`),已经生产`.o`文件,但是没有生成`.out`.出错的原因在于`ld`(链接器),没有完成`a.cpp`和`b.cpp`的链接,什么原因?

> 原因就在于我们在`a.h`的文件中的`global`是定义而非声明,因为`a.cpp`和`b.cpp`都包含了`a.h`文件,而`global`又是定义而非声明,因此说明它被定义了两次,故报错.

既然如此,怎样让`a.h`中的变量`global`变成声明呢?加上`extern`,显然这样能够很好的解决问题.

```c++
//a.h
void f();
extern int global;
```

但是,如果此时我们需要在`b.cpp`文件中去使用这个变量

```c++
//b.cpp
#include "a.h"
void f()
{
    global++;
}
```

然后再次尝试执行命令`g++ a.cpp b.cpp --save-temps`

```sh
alvin@ros:~/cPlus_ws/test$ g++ a.cpp b.cpp --save-temps 
/usr/bin/ld: b.o: in function `f()':
b.cpp:(.text+0xa): undefined reference to `global'
/usr/bin/ld: b.cpp:(.text+0x13): undefined reference to `global'
collect2: error: ld returned 1 exit status
```

报错显示没有定义`undefine`变量`global`,显然,我们`.h`文件仅仅是声明了有`global`这个变量,但是在`b.cpp`中去使用的时候却没有去定义,因此编译器通过了,但`ld`没有找到.

解决方案:修改`b.cpp`

```c++
//b.cpp
#include "a.h"
int global;
void f()
{
    global++;
}
```

### 总结

关于**声明(declaration)** 和**定义(definition)**

- 一个`.cpp`文件就是一个编译单元
- 只有声明的东西才能放入`.h`
  - extern变量
  - 函数的原型
  - 类/`struct `声明

## 易错点二:`.h`的标准头文件结构

### 标准头文件结构的含义

一般情况下,我们的`.h`文件不像是上面定义的那样,格式应该如下:

```cpp
//a.h
#ifndef _A_H_
#define _A_H_
void f();
extern int global;
#endif
```

那么`#`的三行作用是什么作用呢?意思就是如果我们没有定义`_A_H_`,那么我们定义`_A_H_`,直至我们定义为止(`endif`),但是,如果我们先前已经定义了呢?则`#ifndef _A_H_`与`#endif`之间的代码将不会被执行.

比如我们修改`a.h`的文件如下:

```cpp
//a.h
#define _A_H_
#ifndef _A_H_
void f();
extern int global;
#endif
```

显然,我们可以预料到函数f以及声明的变量`global`将不会被执行.

执行指令`g++ a.cpp --save-temps`

查看编译过程文件`a.ii`,显而易见,预料是对的!因此,我们可以初步得知它的表层含义.

```cpp
//a.ii
# 1 "a.cpp"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "/usr/include/stdc-predef.h" 1 3 4
# 1 "<command-line>" 2
# 1 "a.cpp"
# 1 "a.h" 1
# 2 "a.cpp" 2
int main()
{
 return 0;
}
```

### 标准头文件结构的作用

那么它的实际用处是什么呢?

我们重新改写一下`a.h`文件

```cpp
//a.h
void f();
extern int global;
class A{};
```

然后新建一个`b.h`文件,而`b.h`包含了`a.h`的类的变量`a`

```cpp
//b.h
#include "a.h"
extern A a;
```

最后修改一下`a.cpp`, 它由于需求必须引入两个头文件.

```cpp
#include "a.h"
#include "b.h"
int main()
{
        return 0;
}
```

执行命令`g++ a.cpp --save-temps`

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-11_22-18.png"/>

报错显示`error: redefinition of ‘class A’`,重新定义了`classA`这个类

什么原因呢?查看编译过程文件`a.ii`

```cpp
# 1 "a.cpp"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "/usr/include/stdc-predef.h" 1 3 4
# 1 "<command-line>" 2
# 1 "a.cpp"
# 1 "a.h" 1
void f();
extern int global;
class A{};
# 2 "a.cpp" 2
# 1 "b.h" 1
# 1 "a.h" 1
void f();
extern int global;
class A{};
# 2 "b.h" 2

extern A a;
# 3 "a.cpp" 2
int main()
{
 return 0;
}
```

很明显.程序确实在编译过程中定义了两次`classA`这个类

怎么修改?重新修改`a.h`

```cpp
//a.h
#ifndef _A_H_
#define _A_H_
void f();
extern int global;
#endif
```

此时再度执行就会发现,一切正常,查看编译过程文件`a.ii`

```cpp
# 1 "a.cpp"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "/usr/include/stdc-predef.h" 1 3 4
# 1 "<command-line>" 2
# 1 "a.cpp"
# 1 "a.h" 1


void f();
extern int global;
class A{};
# 2 "a.cpp" 2
# 1 "b.h" 1


extern A a;
# 3 "a.cpp" 2
int main()
{
 return 0;
}
```

### 总结

标准头文件结构的作用就是防止同一个`cpp`文件多次调用某一个`.h`文件时,避免头文件里类被反复调用而产生错误.
