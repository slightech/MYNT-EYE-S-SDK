# Tools for MYNT® EYE cameras

## Prerequisites

[OpenCV](https://opencv.org/),

```bash
# Linux, macOS
export OpenCV_DIR=~/opencv

# Windows
set OpenCV_DIR=C:\opencv
```

Python packages,

```bash
cd <sdk>/tools/
sudo pip install -r requirements.txt
```

[ROS](http://www.ros.org/) if using rosbag.

## Build

```bash
cd <sdk>
make tools
```

---

## Record data (mynteye dataset)

```bash
./tools/_output/bin/dataset/record

# Windows
.\tools\_output\bin\dataset\record.bat
```

## Analytics data (mynteye dataset)

### imu_analytics.py

```bash
python tools/analytics/imu_analytics.py -i dataset -c tools/config/mynteye/mynteye_config.yaml \
-al=-1.2,1.2 -gl= -gdu=d -gsu=d -kl=
```

### stamp_analytics.py

```bash
python tools/analytics/stamp_analytics.py -i dataset -c tools/config/mynteye/mynteye_config.yaml
```

---

## Record data (rosbag)

```bash
cd <sdk>
make ros
```

```bash
source wrappers/ros/devel/setup.bash
roslaunch mynt_eye_ros_wrapper mynteye.launch
```

```bash
rosbag record -O mynteye.bag /mynteye/left/image_raw /mynteye/imu/data_raw /mynteye/temperature/data_raw
```

## Analytics data (rosbag)

### imu_analytics.py

```bash
python tools/analytics/imu_analytics.py -i mynteye.bag
```

### stamp_analytics.py

```bash
python tools/analytics/stamp_analytics.py -i mynteye.bag
```

---

## Writer

### device_info_writer.cc

```bash
./tools/_output/bin/writer/device_info_writer tools/writer/config/device.info

# Windows
.\tools\_output\bin\writer\device_info_writer.bat tools\writer\config\device.info
```

### img_params_writer.cc

```bash
./tools/_output/bin/writer/img_params_writer tools/writer/config/img.params

# Windows
.\tools\_output\bin\writer\img_params_writer.bat tools\writer\config\img.params
```

### imu_params_writer.cc

```bash
./tools/_output/bin/writer/imu_params_writer tools/writer/config/imu.params

# Windows
.\tools\_output\bin\writer\imu_params_writer.bat tools\writer\config\imu.params
```

### save_all_infos.cc

```bash
./tools/_output/bin/writer/save_all_infos

# Windows
.\tools\_output\bin\writer\save_all_infos.bat
```

---

## Checksum

```bash
./tools/checksum/md5sum.sh <file or directory>
```
