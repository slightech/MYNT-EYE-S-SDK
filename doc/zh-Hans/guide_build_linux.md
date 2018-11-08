# 编译 on Linux {#guide_build_linux}

> Ubuntu 16.04, Ubuntu 14.04

## 获取代码

```bash
git clone https://github.com/slightech/MYNT-EYE-S-SDK.git
```

## 准备依赖

```bash
cd mynt-eye-s-sdk/
make init
```

### [OpenCV](https://opencv.org/)

编译前，可在系统终端（Windows 命令提示符）里如下指定 OpenCV 路径，其为 `OpenCVConfig.cmake` 目录：

```bash
# Linux, macOS
export OpenCV_DIR=~/opencv

# Windows
set OpenCV_DIR=C:\opencv
```

## 编译代码

```bash
make install
```

结果：

![make install](make_install.png)

\latexonly
\includegraphics[width=0.6\textwidth,keepaspectratio]{make_install.png}
\endlatexonly

> CMake 如何引入编译好的库，可参考 `samples/CMakeLists.txt` 里的配置。
