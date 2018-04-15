# ROS 封装 {#guide_ros}

ROS 封装实现在 ``<sdk>/wrappers/ros`` 目录。

## 依赖

* [ROS](http://www.ros.org/) 环境。

## 编译

```bash
cd <sdk>
make ros
```

## 运行

运行发布节点，

```bash
source wrappers/ros/devel/setup.bash
roslaunch mynt_eye_ros_wrapper mynteye.launch
```

运行发布节点，同时打开 RViz 预览图像，

```bash
source wrappers/ros/devel/setup.bash
roslaunch mynt_eye_ros_wrapper display.launch
```
