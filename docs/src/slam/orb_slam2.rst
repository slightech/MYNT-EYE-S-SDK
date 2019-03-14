.. _slam_orb_slam2:

How to use in `ORB_SLAM2 <https://github.com/raulmur/ORB_SLAM2>`_
==================================================================


If you wanna run ORB_SLAM2 with MYNT EYE camera, please follow the steps:
-------------------------------------------------------------------------

1. Download `MYNT-EYE-S-SDK <https://github.com/slightech/MYNT-EYE-S-SDK.git>`_ and follow steps to install.
2. Follow the normal procedure to install ORB_SLAM2.
3. Update ``distortion_parameters`` and ``projection_parameters`` to ``<ORB_SLAM2>/config/mynteye_*.yaml``.
4. Run examples by MYNT® EYE.

Binocular camera sample
------------------------

* Calibrate a stereo camera with `ROS-StereoCalibration <http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration>`_ or OpenCV, and then update parameters to ``<ORB_SLAM2>/config/mynteye_stereo.yaml``.

* Execute ``build.sh``:

.. code-block:: bash

  chmod +x build.sh
  ./build.sh

* Run stereo sample using the follow type:

.. code-block:: bash

  ./Examples/Stereo/stereo_mynt ./Vocabulary/ORBvoc.txt ./config/mynteye_stereo.yaml true /mynteye/left/image_raw /mynteye/right/image_raw


Building the nodes for mono and stereo (ROS)
--------------------------------------------

* Add the path including ``Examples/ROS/ORB_SLAM2`` to the ``ROS_PACKAGE_PATH`` environment variable. Open ``.bashrc`` file and add at the end the following line. Replace ``PATH`` by the folder where you cloned ORB_SLAM2:

.. code-block:: bash

  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM2/Examples/ROS

* Execute `build_ros.sh`:

.. code-block:: bash

  chmod +x build_ros.sh
  ./build_ros.sh

Mono_ROS Example
~~~~~~~~~~~~~~~~~

  * Update ``distortion_parameters`` and ``projection_parameters`` in ``<ORBSLAM2>/config/mynteye_mono.yaml``

.. code-block:: bash

  cd MYNT-EYE-S-SDK

  ./samples/_output/bin/tutorials/get_img_params

After running the above type, pinhole's ``distortion_parameters`` and ``projection_parameters`` is obtained, and then update to ``<ORB_SLAM2>/config/mynteye_mono.yaml``.

  * Launch ORB_SLAM2 ``Mono_ROS``

.. code-block:: bash

  rosrun ORB_SLAM2 mynteye_mono ./Vocabulary/ORBvoc.txt ./config/mynteye_mono.yaml /mynteye/left/image_raw

Stereo_ROS Example
~~~~~~~~~~~~~~~~~~~

  * Calibrate a stereo camera with `ROS-StereoCalibration <http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration>`_ or OpenCV, and then update parameters to ``<ORB_SLAM2>/config/mynteye_stereo.yaml``.

  * Launch ORB_SLAM2 ``Stereo_ROS``

.. code-block:: bash

  rosrun ORB_SLAM2 ros_mynteye_stereo ./Vocabulary/ORBvoc.txt ./config/mynteye_stereo.yaml true /mynteye/left/image_raw /mynteye/right/image_raw
