.. _wrapper_ros:

How to use ROS
==============

Compile and run the node according to :ref:`sdk_install_ros_wrapper` .

.. tip::

  Before doing below you need open a terminal to launch ros node first

``rostopic list`` lists all released nodes:

.. code-block:: bash

  $ rostopic list
  /mynteye/depth/image_raw
  /mynteye/disparity/image_norm
  /mynteye/disparity/image_raw
  /mynteye/imu/data_raw
  /mynteye/left/camera_info
  /mynteye/left/image_raw
  /mynteye/left/image_rect
  /mynteye/points/data_raw
  /mynteye/right/camera_info
  /mynteye/right/image_raw
  /mynteye/right/image_rect
  /mynteye/temp/data_raw
  ...

``rostopic hz <topic>`` checks the data:

.. code-block:: bash

  $ rostopic hz /mynteye/imu/data_raw
  subscribed to [/mynteye/imu/data_raw]
  average rate: 505.953
    min: 0.000s max: 0.018s std dev: 0.00324s window: 478
  average rate: 500.901
    min: 0.000s max: 0.018s std dev: 0.00327s window: 975
  average rate: 500.375
    min: 0.000s max: 0.019s std dev: 0.00329s window: 1468
  ...

``rostopic echo <topic>`` can print and release data. Please read `rostopic <http://wiki.ros.org/rostopic>`_ for more information.

The ROS file is structured like follows:

.. code-block:: none

  <sdk>/wrappers/ros/
  ├─src/
  │  └─mynt_eye_ros_wrapper/
  │     ├─config/
  │     │  ├─device/
  │     │     ├─standard.yaml   # S1030
  │     │     └─standard2.yaml  # S21X0/S210A
  │     │  ├─laserscan/
  │     │  ├─process/
  │     │  └─...
  │     ├─launch/
  │     │  ├─display.launch
  │     │  └─mynteye.launch
  │     │  └─...
  │     ├─mesh/
  │     ├─rviz/
  │     ├─src/
  |     |  ├─configuru.hpp
  │     │  ├─wrapper_node.cc
  │     │  └─wrapper_nodelet.cc
  │     ├─srv/
  |     |  └─GetInfo.srv
  │     ├─CMakeLists.txt
  │     ├─nodelet_plugins.xml
  │     └─package.xml
  └─README.md

In ``mynteye.launch``, you can configure the topics and frame_ids, decide which data to enable, ``standard.yaml`` (standard2.yaml is S21XX config file) can set parameters for device. Please set ``gravity`` to the local gravity acceleration.


standard.yaml/standard2.yaml:

.. code-block:: xml

  # s21XX modify frame/resolution
  standard2/request_index: 2

  # s1030 modify frame/imu hz
  # standard/frame_rate range: {10,15,20,25,30,35,40,45,50,55,60}
  standard/frame_rate: -1
  # standard/frame_rate: 25

  # standard/imu_frequency range: {100,200,250,333,500}
  standard/imu_frequency: -1
  # standard/imu_frequency: 200
  ...

  # s21X0 modify brightness
  # standard2/brightness range: [1,255]
  standard2/brightness: -1
  # standard2/brightness: 70
  ...

  # s210a modify brightness
  # standard210a/brightness range: [1,255]
  standard210a/brightness: -1
  # standard210a/brightness: 70
  ...


mynteye.launch:

.. code-block:: xml

  <arg name="gravity" default="9.8" />

For printing debug info, replace ``Info`` in ``wrapper_node.cc`` to ``Debug`` :

.. code-block:: c++

  ros::console::set_logger_level(
      ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
