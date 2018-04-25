# 工具 {#guide_tools}

样例在 `<sdk>/tools` 目录，其提供一些有用的工具。

## 依赖

* [OpenCV](https://opencv.org/)，部分工具需要。
  * 编译前，可在系统终端（Windows 命令提示符）里如下指定 OpenCV 路径，其为 `OpenCVConfig.cmake` 目录：

```bash
# Linux, macOS
export OpenCV_DIR=~/opencv

# Windows
set OpenCV_DIR=C:\opencv
```

* Python 第三方库，脚本需要。

```bash
cd tools/
sudo pip install -r requirements.txt
```

## 编译

```bash
make tools
```

## 录制数据集

```bash
./tools/_output/bin/dataset/record

# Windows
.\tools\_output\bin\dataset\record.bat
```

默认录制进 `dataset` 目录，加参数可指定该目录。

## 分析数据集

分析 IMU 数据，

```bash
python tools/analytics/imu_analytics.py -i dataset -c tools/config/mynteye/mynteye_config.yaml \
-al=-1.2,1.2 -gl= -gdu=d -gsu=d -kl=
```

![imu analytics](imu_analytics.png)

\latexonly
\includegraphics[width=0.6\textwidth,keepaspectratio]{imu_analytics.png}
\endlatexonly

分析图像 & IMU 时间戳，

```bash
python tools/analytics/stamp_analytics.py -i dataset -c tools/config/mynteye/mynteye_config.yaml
```

![stamp analytics](stamp_analytics.png)

\latexonly
\includegraphics[width=0.6\textwidth,keepaspectratio]{stamp_analytics.png}
\endlatexonly

> 如果用 ROS ，分析脚本也支持 ROS Bag 格式。

## 结语

设备信息读写、校验码等更多工具的说明，请见 `tools/README.md` 。
