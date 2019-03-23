.. _slam_vins_fusion:

How to use in `VINS-Fusion <https://github.com/HKUST-Aerial-Robotics/Vins-Fusion>`_
====================================================================================


If you wanna run VINS-Fusion with MYNT EYE camera, please follow the steps:
----------------------------------------------------------------------------

1. Download `MYNT-EYE-S-SDK <https://github.com/slightech/MYNT-EYE-S-SDK.git>`_ and install mynt_eye_ros_wrapper.
2. Follow the normal procedure to install VINS-Fusion.
3. Run mynt_eye_ros_wrapper and VINS-Fusion.


Prerequisites
--------------

1. Install Ubuntu 64-bit 16.04 or 18.04. ROS Kinetic or Melodic.(if already installed, please ignore). `ROS Installation <http://wiki.ros.org/ROS/Installation>`_
2. Install `Ceres <http://ceres-solver.org/installation.html>`_


Install MYNT-EYE-VINS-FUSION-Samples
-------------------------------------

.. code-block:: bash

  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  git clone -b mynteye https://github.com/slightech/MYNT-EYE-VINS-FUSION-Samples.git
  cd ..
  catkin_make
  source ~/catkin_ws/devel/setup.bash

(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)

Run VINS-FUSION with MYNT® EYE
-------------------------------

1. Launch mynteye node

.. code-block:: bash

  cd (local path of MYNT-EYE-S-SDK)
  source ./wrappers/ros/devel/setup.bash
  roslaunch mynt_eye_ros_wrapper mynteye.launch

2. Open another terminal and run vins

.. code-block:: bash

  cd ~/catkin_ws
  roslaunch vins mynteye-s-mono-imu.launch  # mono+imu fusion
  # roslaunch vins mynteye-s-stereo.launch  # Stereo fusion / Stereo+imu fusion
  # roslaunch vins mynteye-avarta-mono-imu.launch  # mono+imu fusion with mynteye-avarta
  # roslaunch vins mynteye-avarta-stereo.launch  # Stereo fusion / Stereo+imu fusion with mynteye-avarta
