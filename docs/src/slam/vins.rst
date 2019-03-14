.. _slam_vins:

How to use in `VINS-Mono <https://github.com/HKUST-Aerial-Robotics/VINS-Mono>`_
================================================================================


If you wanna run VINS-Mono with MYNT EYE camera, please follow the steps:
--------------------------------------------------------------------------

1. Download `MYNT-EYE-S-SDK <https://github.com/slightech/MYNT-EYE-S-SDK.git>`_ and install mynt_eye_ros_wrapper.
2. Follow the normal procedure to install VINS-Mono.
3. Update ``distortion_parameters`` and ``projection_parameters`` to `here <https://github.com/slightech/MYNT-EYE-VINS-Sample/blob/mynteye-s/config/mynteye/mynteye_config.yaml>`_ .
4. Run mynt_eye_ros_wrapper and VINS-Mono.

Install ROS Kinetic conveniently (if already installed, please ignore)
----------------------------------------------------------------------

.. code-block:: bash

  cd ~
  wget https://raw.githubusercontent.com/oroca/oroca-ros-pkg/master/ros_install.sh && \
  chmod 755 ./ros_install.sh && bash ./ros_install.sh catkin_ws kinetic

Install MYNT-EYE-VINS-Sample
------------------------------

.. code-block:: bash

  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  git clone -b mynteye-s https://github.com/slightech/MYNT-EYE-VINS-Sample.git
  cd ..
  catkin_make
  source devel/setup.bash
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
  source ~/.bashrc

Get image calibration parameters
---------------------------------

Use MYNT® EYE's left eye camera and IMU. By `MYNT-EYE-S-SDK <https://github.com/slightech/MYNT-EYE-S-SDK.git>`_ API ``GetIntrinsics()`` function and ``GetExtrinsics()`` function, you can "get the image calibration parameters of the current working device:

.. code-block:: bash

  cd MYNT-EYE-S-SDK
  ./samples/_output/bin/tutorials/get_img_params

After running the above type, pinhole's ``distortion_parameters`` and ``projection_parameters`` is obtained , and then update to `here <https://github.com/slightech/MYNT-EYE-VINS-Sample/blob/mynteye-s/config/mynteye/mynteye_config.yaml>`_ .

Run VINS-Mono with MYNT® EYE
-----------------------------

.. code-block:: bash

  cd ~/catkin_ws
  roslaunch mynt_eye_ros_wrapper mynteye.launch
  roslaunch vins_estimator mynteye.launch

.. note::

  If you want to use a fish-eye camera model, please click `here <https://github.com/slightech/MYNT-EYE-VINS-Sample/tree/mynteye-s/calibration_images>`_ .
