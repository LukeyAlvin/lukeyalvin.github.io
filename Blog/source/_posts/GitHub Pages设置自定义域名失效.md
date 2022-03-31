---
title: GitHub Pages设置自定义域名失效
tags: [GitHub,Hexo]
comments: true
mathjax: true
data: 2021-10-30 10:44:42
categories: 经验
---
在GitHub Pages设置自定义域名之后，发现每次hexo d 后都会失效，又要重新设置，太麻烦了。
<!--more-->
只要在source 目录添加一个新文件CNAME就好
CNAME –不带任何后缀，这就是全称，里面写的是你的域名
然后就ok了。
怎么push都不用再去GitHub Pages设置了
