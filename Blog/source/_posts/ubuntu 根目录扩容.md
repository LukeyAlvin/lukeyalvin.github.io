> 本文由 [简悦 SimpRead](http://ksria.com/simpread/) 转码， 原文地址 [blog.csdn.net](https://blog.csdn.net/weixin_41018348/article/details/82592057#commentBox)

本人 ubuntu 系统安了 cuda 一些乱七八糟的东西后，发现根目录空间不够了（15G，之前安装双系统设置的），痛下决心把根目录给扩容一下（主要是怕系统崩了），下边介绍一下使用 gparted 分区软件对 linux 系统进行磁盘分区。（亲测好使！！！扩容后应该能够折腾一段时间了）

在 ubuntu16.04 下，使用下面命令安装 gparted:

sudo apt-get install gparted

安装好后用下边命令启动 gparted:

sudo gparted

![](https://img-blog.csdn.net/20180910164650905?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTAxODM0OA==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

在 ubuntu16.04 安装好后启动的界面如上，可以看到开辟的空间都带有钥匙标记，这时不能对分区进行操作，需要先卸载（unmount）或者停止（swapoff）（附：没试过直接在现已安装的 Ubuntu 下卸载或停止，而是用制作 Ubuntu 启动 U 盘试用模式下进行的），用 U 盘 Ubuntu 启动盘进入试用模式。

在试用系统中，搜索 gparted，就能启动已安装的 gparted

![](https://img-blog.csdn.net/2018091016554517?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTAxODM0OA==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

进入 gparted 后，可以看到除了 linux-swap 交互空间的钥匙标记还有，其它的都没了，这时需要将 linux-swap 停止，鼠标右键选择 swapoff 后，可以看到所有分区的钥匙标记都没了。

本人的 home（sda7）分区闲置的空间比较大，选择在 home 分区进行压缩出新的空闲空间。在 sda7 中选择 Resize/Move 进行压缩

![](https://img-blog.csdn.net/20180910170314360?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTAxODM0OA==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

然后选择要压缩的空间，其中有三个编辑框，分别是：Free Space Preceding, New Size, Free Space following

Free Space Preceding 代表从 sda7 压缩 N MB，在 sda7 的上方，即 sda6 与 sda7 之间

New Size 表示当前分区的容量，若要压缩该分区，该值需要减去压缩值

Free Space following 代表从 sda7 压缩 N MB，在 sda7 的下方

![](https://img-blog.csdn.net/20180910170637122?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTAxODM0OA==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

选择好 free space prceding 后，点击一下 New size 的数值，会自动计算好压缩完后的空间，然后点击 Resize/Move 就能看到在 sda7 上方会压缩出一个新分区 unallocated。

下面需要将 unallocated，移动到 sda1 下边或上边才能对其扩容。

移动的方法与压缩一样，选择 sda6 进行 resize/Move，进入界面后直接进行 resize/move，就能将 unallocated 移动到 sda6 上方，同理对 sda5 进行操作

![](https://img-blog.csdn.net/20180910171555187?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTAxODM0OA==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

这时 unallocated 到了 sda2 下方，发现对 sda2 不能进行同样的操作。仔细看了一下，原来系统是将整个硬盘分成了 sda1 和 sda2 两个分区，然后再将 sda2 分成了三个分区 sda5\sda6sda7

![](https://img-blog.csdn.net/20180910172018778?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTAxODM0OA==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

实际上我们需要的是将 sda2 压缩出一个新的空闲分区给 sda1

这时，对 sda2 重复 sda7 的操作，就能将 unallocated 移动到 sda2 的上方（即 sda1 下方）。

选择 sda1 进行 rezise/move，对这个滑条进行操作，拉满即可（实际上就是对 sda1 扩容）

![](https://img-blog.csdn.net/20180910172439184?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTAxODM0OA==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

完成后就能看到 sda1 已经扩容至所设容量（下边图是已扩容好的）

![](https://img-blog.csdn.net/20180910172747777?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTAxODM0OA==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

最后，一定要记得应用到整个系统，

即选择菜单栏 >> Edit >> Apply all Operations >> Apply

等待完成即可。（本人操作完成时有两个 warning，并没理会，重启系统后一切正常）

PS: 由于本人是在系统扩容后才写的博客（主要是怕自己需要用的时候忘了操作流程），上边的图都是在根目录扩容后的，要是看得不明白，还请多多包涵～

参考教程：

[https://blog.csdn.net/code_segment/article/details/79237500](https://blog.csdn.net/code_segment/article/details/79237500)

[https://blog.csdn.net/t765833631/article/details/79031063](https://blog.csdn.net/t765833631/article/details/79031063)