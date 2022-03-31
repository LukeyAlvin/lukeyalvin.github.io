---
title: gazebo加载world出现黑屏问题
tags: [BUG,gazebo]
date: {{ date }}
comments: true
mathjax: true
categories: ROS
---

{%cq%}
    近期发在gazebo在加载一些world环境时,总是出现下面黑屏的情况,这是因为model库加载不正确导致的{%endcq%}

<!-- more -->

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-02_22-44.png"/>

解决方法： 通过直接下载所有模型到用户的根目录下的.gazebo/models/下 

```bash
$ cd ~/.gazebo/
$ mkdir -p models
$ cd ~/.gazebo/models/
$ wget http://file.ncnynl.com/ros/gazebo_models.txt
$ wget -i gazebo_models.txt
$ ls model.tar.g* | xargs -n1 tar xzvf
```

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-02_22-49.png"/>

完美解决

<img src="https://gitee.com/LukeyAlvin/img_mk/raw/master/img/2021-11-02_23-16.png"/>