# MYNT® EYE S SDK

################################################################################
Language: 简体中文
################################################################################

## 如何开始使用 SDK

1) 运行样例程序

安装完 SDK 的 exe 安装包后，桌面会生成 SDK 根目录的快捷方式。

进入 "<SDK_ROOT_DIR>\bin\samples" 目录，双击 "camera_with_senior_api.exe" 运行，即可看到双目实时画面。

2）生成样例工程

首先，安装好 Visual Studio 2017 <https://visualstudio.microsoft.com/> 和 CMake <https://cmake.org/> 。

接着，进入 "<SDK_ROOT_DIR>\samples" 目录， 双击 "generate.bat" 即可生成样例工程。

p.s. 样例教程，可见 https://slightech.github.io/MYNT-EYE-S-SDK/ 主页给出的 Guide 文档。

p.p.s. 运行结果，参考下方英文内容。

3）如何于 Visual Studio 2017 下使用 SDK

进入 "<SDK_ROOT_DIR>\samples\simple_demo\project_vs2017" ，见 "README.md" 说明。

################################################################################
Language: English
################################################################################

## How to start using SDK

1) Run the prebuilt samples, ensure the SDK works well.

After you install the win pack of SDK, there will be a shortcut to the SDK root directory on your desktop.

First, you should plug the MYNT® EYE camera in a USB 3.0 port.

Second, goto the "<SDK_ROOT_DIR>\bin\samples" directory and click "camera_with_senior_api.exe" to run.

Finally, you will see the window that display the realtime frame of the camera.

2) Generate samples project of Visual Studio 2017.

First, you should install Visual Studio 2017 <https://visualstudio.microsoft.com/> and CMake <https://cmake.org/>.

Second, goto the "<SDK_ROOT_DIR>\samples" directory and click "generate.bat" to run.

Finally, you could click `_build\mynteye_samples.sln` to open the samples project.

p.s. The tutorials of samples are here: https://slightech.github.io/MYNT-EYE-S-SDK-Guide/src/data/contents.html.

p.p.s. The example result of "generate.bat",

```cmd
-- The C compiler identification is MSVC 19.14.26429.4
-- The CXX compiler identification is MSVC 19.14.26429.4
-- Check for working C compiler: C:/Program Files (x86)/Microsoft Visual Studio/2017/Community/VC/Tools/MSVC/14.14.26428/bin/Hostx86/x64/cl.exe
-- Check for working C compiler: C:/Program Files (x86)/Microsoft Visual Studio/2017/Community/VC/Tools/MSVC/14.14.26428/bin/Hostx86/x64/cl.exe -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working CXX compiler: C:/Program Files (x86)/Microsoft Visual Studio/2017/Community/VC/Tools/MSVC/14.14.26428/bin/Hostx86/x64/cl.exe
-- Check for working CXX compiler: C:/Program Files (x86)/Microsoft Visual Studio/2017/Community/VC/Tools/MSVC/14.14.26428/bin/Hostx86/x64/cl.exe -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- HOST_ARCH: x86_64
-- OpenCV ARCH: x64
-- OpenCV RUNTIME: vc15
-- OpenCV STATIC: OFF
-- Found OpenCV: C:/Users/John/AppData/Roaming/Slightech/MYNTEYES/SDK/2.2.1/3rdparty/opencv/build (found version "3.4.3")
-- Found OpenCV 3.4.3 in C:/Users/John/AppData/Roaming/Slightech/MYNTEYES/SDK/2.2.1/3rdparty/opencv/build/x64/vc15/lib
-- You might need to add C:\Users\John\AppData\Roaming\Slightech\MYNTEYES\SDK\2.2.1\3rdparty\opencv\build\x64\vc15\bin to your PATH to be able to run your applications.
-- Found OpenCV: 3.4.3
CMake Warning at C:/Program Files/CMake/share/cmake-3.10/Modules/FindBoost.cmake:567 (message):
  Imported targets and dependency information not available for Boost version
  (all versions older than 1.33)
Call Stack (most recent call first):
  C:/Program Files/CMake/share/cmake-3.10/Modules/FindBoost.cmake:907 (_Boost_COMPONENT_DEPENDENCIES)
  C:/Program Files/CMake/share/cmake-3.10/Modules/FindBoost.cmake:1542 (_Boost_MISSING_DEPENDENCIES)
  C:/Users/John/AppData/Roaming/Slightech/MYNTEYES/SDK/2.2.1/cmake/Option.cmake:47 (find_package)
  CMakeLists.txt:26 (include)


-- Could NOT find Boost
--
-- Platform:
--   HOST_OS: Win
--   HOST_NAME: Win
--   HOST_ARCH: x86_64
--   HOST_COMPILER: MSVC
--     COMPILER_VERSION: 19.14.26429.4
--     COMPILER_VERSION_MAJOR: 19
--     COMPILER_VERSION_MINOR: 14
--     COMPILER_VERSION_PATCH: 26429
--     COMPILER_VERSION_TWEAK: 4
--   CUDA_VERSION: 9.2
--     CUDA_VERSION_MAJOR: 9
--     CUDA_VERSION_MINOR: 2
--     CUDA_VERSION_STRING: 9.2
--   OpenCV_VERSION: 3.4.3
--     OpenCV_VERSION_MAJOR: 3
--     OpenCV_VERSION_MINOR: 4
--     OpenCV_VERSION_PATCH: 3
--     OpenCV_VERSION_TWEAK: 0
--     OpenCV_VERSION_STATUS:
--     OpenCV_WITH_WORLD: TRUE
--
-- Options:
--   WITH_API: ON
--     OpenCV: YES
--     OpenCV_VERSION: 3.4.3
--     OpenCV_WORLD: YES
--   WITH_DEVICE_INFO_REQUIRED: ON
--   WITH_BOOST: ON
--     Boost: NO
--   WITH_GLOG: OFF
--
-- Features:
--   Filesystem: native
--
-- Visual Studio >= 2010, MSVC >= 10.0
-- C_FLAGS: /DWIN32 /D_WINDOWS /W3 -Wall -march=native
-- CXX_FLAGS: /DWIN32 /D_WINDOWS /W3 /GR /EHsc -Wall -march=native
-- Found mynteye: 2.2.1
-- Generating camera_a.bat
-- Generating get_depth_with_region.bat
-- Generating camera_d.bat
-- Generating camera_u.bat
CMake Warning at tutorials/CMakeLists.txt:70 (find_package):
  By not providing "FindPCL.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "PCL", but
  CMake did not find one.

  Could not find a package configuration file provided by "PCL" with any of
  the following names:

    PCLConfig.cmake
    pcl-config.cmake

  Add the installation prefix of "PCL" to CMAKE_PREFIX_PATH or set "PCL_DIR"
  to a directory containing one of the above files.  If "PCL" provides a
  separate development package or SDK, be sure it has been installed.


CMake Warning at tutorials/CMakeLists.txt:86 (message):
  PCL not found :(


-- Generating get_device_info.bat
-- Generating get_img_params.bat
-- Generating get_imu_params.bat
-- Generating get_stereo.bat
-- Generating get_stereo_rectified.bat
-- Generating get_disparity.bat
-- Generating get_depth.bat
-- Generating get_imu.bat
-- Generating get_from_callbacks.bat
-- Generating get_with_plugin.bat
-- Generating ctrl_framerate.bat
-- Generating ctrl_auto_exposure.bat
-- Generating ctrl_manual_exposure.bat
-- Generating ctrl_infrared.bat
-- Generating get_all_device_info.bat
-- Configuring done
-- Generating done
-- Build files have been written to: C:/Users/John/AppData/Roaming/Slightech/MYNTEYES/SDK/2.2.1/samples/_build
Press any key to continue . . .
```

3) Start using MYNT® EYE S SDK with Visual Studio 2017

Goto the "<SDK_ROOT_DIR>\samples\simple_demo\project_vs2017", see the "README.md".
