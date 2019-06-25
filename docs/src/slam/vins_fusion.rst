.. _vins_fusion:

How To Use In `VINS-Fusion <https://github.com/HKUST-Aerial-Robotics/Vins-Fusion>`_
====================================================================================


If you wanna run VINS-Fusion with MYNT EYE camera, please follow the steps:
----------------------------------------------------------------------------

1. Download `MYNT-EYE-S-SDK <https://github.com/slightech/MYNT-EYE-S-SDK.git>`_ and install mynt_eye_ros_wrapper.
2. Follow the normal procedure to install VINS-Fusion.
3. Run mynt_eye_ros_wrapper and VINS-Fusion.

Install ROS Kinetic conveniently (if already installed, please ignore)
----------------------------------------------------------------------

.. code-block:: bash

  cd ~
  wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
  chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws kinetic

Install Ceres
--------------

.. code-block:: bash

  cd ~
  git clone https://ceres-solver.googlesource.com/ceres-solver
  sudo apt-get -y install cmake libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
  sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
  sudo apt-get update && sudo apt-get install libsuitesparse-dev
  mkdir ceres-bin
  cd ceres-bin
  cmake ../ceres-solver
  make -j3
  sudo make install

Install MYNT-EYE-VINS-FUSION-Samples
-------------------------------------

.. code-block:: bash

  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  git clone https://github.com/slightech/MYNT-EYE-VINS-FUSION-Samples.git
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
  roslaunch mynt_eye_ros_wrapper vins_fusion.launch

2. Open another terminal and run vins

.. code-block:: bash

  cd ~/catkin_ws/src
  source ./devel/setup.bash
  roslaunch vins mynteye-s-stereo.launch  # Stereo fusion / Stereo+imu fusion
  # roslaunch vins mynteye-s-mono-imu.launch  # mono+imu fusion
  # roslaunch vins mynteye-s2100-mono-imu.launch  # mono+imu fusion with mynteye-s2100
  # roslaunch vins mynteye-s2100-stereo.launch  # Stereo fusion / Stereo+imu fusion with mynteye-s2100
