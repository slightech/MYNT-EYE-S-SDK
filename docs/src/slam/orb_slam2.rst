.. _orb_slam2:

How To Use In `ORB_SLAM2 <https://github.com/raulmur/ORB_SLAM2>`_
==================================================================


If you wanna run ORB_SLAM2 with MYNT EYE camera, please follow the steps:
-------------------------------------------------------------------------

1. Download `MYNT-EYE-S-SDK <https://github.com/slightech/MYNT-EYE-S-SDK.git>`_ and follow steps to install.
2. Follow the normal procedure to install ORB_SLAM2.
3. Run examples by MYNT® EYE.

Prerequisites
--------------

.. code-block:: bash

  sudo apt-get -y install libglew-dev cmake libgoogle-glog-dev
  cd ~
  git clone https://github.com/stevenlovegrove/Pangolin.git
  cd Pangolin
  mkdir build
  cd build
  cmake ..
  cmake --build .
  sudo make install

Building the nodes for mono and stereo (ROS)
--------------------------------------------

* Add the path including ``Examples/ROS/ORB_SLAM2`` to the ``ROS_PACKAGE_PATH`` environment variable. Open ``.bashrc`` file and add at the end the following line.

.. code-block:: bash

  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/catkin_ws/src/MYNT-EYE-ORB-SLAM2-Sample

* Execute `build_ros.sh`:

.. code-block:: bash

  chmod +x build.sh
  ./build.sh
  chmod +x build_ros.sh
  ./build_ros.sh


Stereo_ROS Example
~~~~~~~~~~~~~~~~~~~

  * Launch ORB_SLAM2 ``Stereo_ROS``

1. Launch mynteye node

.. code-block:: bash

  cd [path of mynteye-s-sdk]
  make ros
  source ./wrappers/ros/devel/setup.bash
  roslaunch mynt_eye_ros_wrapper mynteye.launch

2. Open another terminal and run ORB_SLAM2

.. code-block:: bash

  rosrun ORB_SLAM2 mynteye_s_stereo ./Vocabulary/ORBvoc.txt ./config/mynteye_s_stereo.yaml false /mynteye/left_rect/image_rect /mynteye/right_rect/image_rect
