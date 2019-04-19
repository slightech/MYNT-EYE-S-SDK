.. _slam_vins:

How to use in `VINS-Mono <https://github.com/HKUST-Aerial-Robotics/VINS-Mono>`_
================================================================================


If you wanna run VINS-Mono with MYNT EYE camera, please follow the steps:
--------------------------------------------------------------------------

1. Download `MYNT-EYE-S-SDK <https://github.com/slightech/MYNT-EYE-S-SDK.git>`_ and install mynt_eye_ros_wrapper.
2. Follow the normal procedure to install VINS-Mono.
3. Run mynt_eye_ros_wrapper and VINS-Mono.

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

Install MYNT-EYE-VINS-Sample
------------------------------

.. code-block:: bash

  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  git clone https://github.com/slightech/MYNT-EYE-VINS-Sample.git
  cd ..
  catkin_make
  source devel/setup.bash
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
  source ~/.bashrc

(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)

Run VINS-Mono with MYNT® EYE
-----------------------------

1. Launch mynteye node

.. code-block:: bash

  cd (local path of MYNT-EYE-S-SDK)
  source ./wrappers/ros/devel/setup.bash
  roslaunch mynt_eye_ros_wrapper mynteye.launch

2. Open another terminal and run vins

.. code-block:: bash

  cd ~/catkin_ws
  roslaunch vins_estimator mynteye_s.launch
