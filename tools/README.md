# Tools for MYNT® EYE tools

## Prerequisites

Ubuntu 16.04, ROS Kinetic.

```bash
cd <sdk>/tools/
sudo pip install -r requirements.txt
```

---

## Record data (mynteye dataset)

```bash
cd <sdk>
make tools
```

```bash
./tools/_output/bin/dataset/record
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
rosbag record -O mynteye.bag /mynteye/left /mynteye/imu /mynteye/temp
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
