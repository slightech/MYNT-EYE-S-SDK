# Samples for MYNT® EYE tools

## Prerequisites

Ubuntu 16.04, ROS Kinetic.

```bash
cd <sdk>/tools/
sudo pip install -r requirements.txt
```

## Record data

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

## Analytics

### imu_analytics.py

```bash
python tools/analytics/imu_analytics.py -i mynteye.bag
```

### stamp_analytics.py

```bash
python tools/analytics/stamp_analytics.py -i mynteye.bag
```
