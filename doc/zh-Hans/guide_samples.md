# 样例 {#guide_samples}

样例在 `<sdk>/samples` 目录，其提供了不同接口层的使用范例。

## 依赖

* [OpenCV](https://opencv.org/)，用于显示图像。
  * 编译前，可在系统终端（Windows 命令提示符）里如下指定 OpenCV 路径，其为 `OpenCVConfig.cmake` 目录：

```bash
# Linux, macOS
export OpenCV_DIR=~/opencv

# Windows
set OpenCV_DIR=C:\opencv
```

## 编译

```bash
make samples
```

## 运行

运行 `api` 层接口样例，显示图像并输出 IMU 。

```bash
./samples/_output/bin/api/camera_a

# Windows
.\samples\_output\bin\api\camera_a.bat
```

运行 `device` 层接口样例，显示图像并输出 IMU 。

```bash
./samples/_output/bin/device/camera_d

# Windows
.\samples\_output\bin\device\camera_d.bat
```

## 结语

更多样例的说明，请见 `samples/README.md` 。
