# Wrappers for MYNT® EYE SDK

## MYNT EYE ROS Wrapper

### Build

```bash
cd <sdk>
make ros
```

### Run

```bash
source wrappers/ros/devel/setup.bash
roslaunch mynt_eye_ros_wrapper mynteye.launch
```

With RViz to preview,

```bash
source wrappers/ros/devel/setup.bash
roslaunch mynt_eye_ros_wrapper display.launch
```
