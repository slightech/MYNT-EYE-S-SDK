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

Install Docker
---------------

.. code-block:: bash

  sudo apt-get update
  sudo apt-get install \
      apt-transport-https \
      ca-certificates \
      curl \
      gnupg-agent \
      software-properties-common
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
  sudo add-apt-repository \
     "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
     $(lsb_release -cs) \
     stable"
  sudo apt-get update
  sudo apt-get install docker-ce docker-ce-cli containerd.io

Then add your account to ``docker`` group by ``sudo usermod -aG docker $YOUR_USER_NAME`` . Relaunch the terminal or logout and re-login if you get ``Permission denied`` error.

To complie with docker,we recommend that you should use more than 16G RAM, or ensure that the RAM and virtual memory space is greater than 16G.


Install MYNT-EYE-VINS-FUSION-Samples
-------------------------------------

.. code-block:: bash

  git clone https://github.com/slightech/MYNT-EYE-VINS-FUSION-Samples.git
  cd MYNT-EYE-VINS-FUSION-Samples/docker
  make build

Note that the docker building process may take a while depends on your network and machine. After VINS-Mono successfully started, open another terminal and play your bag file, then you should be able to see the result. If you need modify the code, simply run ``./run.sh LAUNCH_FILE_NAME`` after your changes.

Run VINS-FUSION with MYNT® EYE
-------------------------------

1. Launch mynteye node

.. code-block:: bash

  cd (local path of MYNT-EYE-S-SDK)
  source ./wrappers/ros/devel/setup.bash
  roslaunch mynt_eye_ros_wrapper vins_fusion.launch

2. Open another terminal and run vins

.. code-block:: bash

  cd path/to/MYNT-EYE-VINS-FUSION-Samples/docker
  ./run.sh mynteye-s/mynt_stereo_imu_config.yaml  # Stereo fusion
  # ./run.sh mynteye-s2100/mynt_stereo_config.yaml # Stereo fusion with mynteye-s2100
  # ./run.sh mynteye-s2100/mynt_stereo_imu_config.yaml # Stereo+imu fusion with mynteye-s2100
