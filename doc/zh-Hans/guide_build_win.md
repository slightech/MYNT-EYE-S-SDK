# 编译 on Windows {#guide_build_win}

> Windows 10

## 前提条件

* [Git](https://git-scm.com/downloads)，用于获取代码。
* [CMake](https://cmake.org/download/)，用于构建编译。
* [Doxygen](http://www.stack.nl/~dimitri/doxygen/download.html)，用于生成文档。

最终，命令提示符（Command Prompt, cmd）里可找到如下命令：

```cmd
>cmake --version
cmake version 3.10.1

>git --version
git version 2.11.1.windows.1

>doxygen --version
1.8.13
```

* [Visual Studio](https://www.visualstudio.com/)
  * [Visual Studio 2015](https://my.visualstudio.com/Downloads?q=Visual Studio 2015)
  * [Visual Studio 2017](https://my.visualstudio.com/Downloads?q=Visual Studio 2017)
* [Windows 10 SDK](https://developer.microsoft.com/en-US/windows/downloads/windows-10-sdk)

以 Visual Studio 2015 举例，请在系统环境变量 `PATH` 里添加上如下路径：

    C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\bin
    C:\Program Files (x86)\MSBuild\14.0\Bin

最终，命令提示符（Command Prompt, cmd）里可找到如下命令：

```cmd
>cl
Microsoft (R) C/C++ Optimizing Compiler Version 19.00.24215.1 for x86

>link
Microsoft (R) Incremental Linker Version 14.00.24215.1

>lib
Microsoft (R) Library Manager Version 14.00.24215.1

>msbuild
Microsoft (R) 生成引擎版本 14.0.25420.1
```

* [MSYS2](http://www.msys2.org/)
  * [国内镜像](https://lug.ustc.edu.cn/wiki/mirrors/help/msys2)
  * [pacman](https://wiki.archlinux.org/index.php/pacman)

打开 MSYS2 MSYS ，然后执行：

```msys
$ pacman -Syu
$ pacman -S make
```

并在系统环境变量 `PATH` 里添加上如下路径：

    C:\msys64\usr\bin

最终，命令提示符（Command Prompt, cmd）里可找到如下命令：

```cmd
>make --version
GNU Make 4.2.1
```

## 获取代码

```cmd
>git clone https://github.com/slightech/MYNT-EYE-S-SDK.git
```

## 准备依赖

```cmd
>cd mynt-eye-s-sdk
>make init
Make init
Init deps
Install cmd: pacman -S
Install deps: git clang-format
pacman -S clang-format (not exists)
error: target not found: clang-format
pip install --upgrade autopep8 cpplint pylint requests
...
Init git hooks
ERROR: clang-format-diff is not installed!
Expect cmake version >= 3.0
cmake version 3.10.1
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

```cmd
>make install
```
