.. _sdk_install_ros_wrapper:

ROS Wrapper Installation
========================

.. only:: html

  =============== ===============
  ROS Kinetic     ROS Indigo
  =============== ===============
  |build_passing| |build_passing|
  =============== ===============

  .. |build_passing| image:: https://img.shields.io/badge/build-passing-brightgreen.svg?style=flat

.. only:: latex

  =============== ===============
  ROS Kinetic     ROS Indigo
  =============== ===============
  ✓               ✓
  =============== ===============

Prepare Environment
--------------------

* `ROS <http://www.ros.org/>`_

ROS Melodic (Ubuntu 18.04)
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
  sudo apt update
  sudo apt install ros-melodic-desktop-full
  sudo rosdep init
  rosdep update

ROS Kinetic (Ubuntu 16.04)
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

  wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
  chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws kinetic

ROS Indigo (Ubuntu 14.04)
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

  wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
  chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws indigo

Compiling Code
--------------

.. code-block:: bash

  cd <sdk>
  make ros

Running node
------------

.. code-block:: bash

  source wrappers/ros/devel/setup.bash
  roslaunch mynt_eye_ros_wrapper mynteye.launch  # this node doesn't have preview

Run the node, and preview by RViz:

.. code-block:: bash

  source wrappers/ros/devel/setup.bash
  roslaunch mynt_eye_ros_wrapper display.launch

Testing Services
-----------------

Run the node as follows, provide device information getting service, see follows:

.. code-block:: bash

  $ source wrappers/ros/devel/setup.bash
  $ rosrun mynt_eye_ros_wrapper get_device_info.py
  LENS_TYPE: 0000
  SPEC_VERSION: 1.0
  NOMINAL_BASELINE: 120
  HARDWARE_VERSION: 2.0
  IMU_TYPE: 0000
  SERIAL_NUMBER: 0610243700090720
  FIRMWARE_VERSION: 2.0
  DEVICE_NAME: MYNT-EYE-S1000

Common issues - ROS Indigo
--------------------------

Cannot find ``libopencv`` while ``make ros``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

  make[3]: *** No rule to make target `/usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8', needed by `/home/john/Workspace/MYNT-EYE-S-SDK/wrappers/ros/devel/lib/libmynteye_wrapper.so'.  Stop.

**Solution 1)** Install OpenCV 2:

.. code-block:: bash

  sudo apt-get update
  sudo apt-get install libcv-dev

**Solution 2)** Install OpenCV 3 & re-compiled ``cv_bridge``:

.. code-block:: bash

  sudo apt-get install ros-indigo-opencv3

  git clone https://github.com/ros-perception/vision_opencv.git
  mv vision_opencv/cv_bridge/ MYNT-EYE-S-SDK/wrappers/ros/src/

Then run ``make ros`` again

Conclusion
-----------

About more details, check the :ref:`wrapper_ros` .
