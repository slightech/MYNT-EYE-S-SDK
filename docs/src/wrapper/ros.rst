.. _wrapper_ros:

How to use ROS
==============

Compile and run the node according to :ref:`sdk_install_ros` .

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
  │     ├─launch/
  │     │  ├─display.launch
  │     │  └─mynteye.launch
  │     ├─msg/
  │     ├─rviz/
  │     ├─src/
  │     │  ├─wrapper_node.cc
  │     │  └─wrapper_nodelet.cc
  │     ├─CMakeLists.txt
  │     ├─nodelet_plugins.xml
  │     └─package.xml
  └─README.md

In ``mynteye.launch``, you can configure the topics and frame_ids, decide which data to enable, and set the control options. Please set ``gravity`` to the local gravity acceleration.

.. code-block:: xml

  <arg name="gravity" default="9.8" />

For printing debug info, replace ``Info`` in ``wrapper_node.cc`` to ``Debug`` :

.. code-block:: c++

  ros::console::set_logger_level(
      ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
