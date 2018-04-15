# 工具 {#guide_tools}

样例在 `<sdk>/tools` 目录，其提供一些有用的工具。

## 准备

```bash
make tools

cd tools/
sudo pip install -r requirements.txt
```

## 录制数据集

```bash
./tools/_output/bin/dataset/record
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

## 结语

如果用 ROS ，分析脚本也支持 ROS Bag 格式。关于 ROS 的录制和分析，请见 `tools/README.md` 。
