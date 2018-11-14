# MYNT® EYE ROS Wrapper

## Prerequisites

* [ROS](http://www.ros.org/)

How to install ROS Kinetic (Ubuntu 16.04),

```bash
wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws kinetic
```

## Build

```bash
cd <sdk>
make ros
```

## Run

```bash
source wrappers/ros/devel/setup.bash
roslaunch mynt_eye_ros_wrapper mynteye.launch
```

With RViz to preview,

```bash
source wrappers/ros/devel/setup.bash
roslaunch mynt_eye_ros_wrapper display.launch
```

## Test

Test `get_info` service,

```bash
source wrappers/ros/devel/setup.bash
rosrun mynt_eye_ros_wrapper get_device_info.py
```

## ROS Indigo

How to install ROS Indigo (Ubuntu 14.04),

```bash
wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws indigo
```

If `libopencv` not found when `make ros`,

    make[3]: *** No rule to make target `/usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8', needed by `/home/john/Workspace/mynt-eye-s-sdk/wrappers/ros/devel/lib/libmynteye_wrapper.so'.  Stop.

Solution 1) Install OpenCV 2 package:

```
sudo apt-get update
sudo apt-get install libcv-dev
```

Solution 2) Install OpenCV 3 and recompile `cv_bridge`:

```
sudo apt-get install ros-indigo-opencv3

git clone https://github.com/ros-perception/vision_opencv.git
mv vision_opencv/cv_bridge/ mynt-eye-s-sdk/wrappers/ros/src/
```

Finally, `make ros` again.

<!--
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
-->
