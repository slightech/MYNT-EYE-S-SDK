.. _sdk_install_ubuntu_src:

Ubuntu Source Installation
==============================

.. only:: html

  =============== =============== ===============
  Ubuntu 14.04    Ubuntu 16.04    Ubuntu 18.04
  =============== =============== ===============
  |build_passing| |build_passing| |build_passing|
  =============== =============== ===============

  .. |build_passing| image:: https://img.shields.io/badge/build-passing-brightgreen.svg?style=flat

.. only:: latex

  =============== =============== ===============
  Ubuntu 14.04    Ubuntu 16.04    Ubuntu 18.04
  =============== =============== ===============
  ✓               ✓               ✓
  =============== =============== ===============

.. tip::

  If you used any other Linux distributions without apt-get package manager, you need to install required packages manaully instead of using ``make init``.

  ============= =====================================================================
  Linux         How to install required packages
  ============= =====================================================================
  Debian based  sudo apt-get install build-essential cmake git libv4l-dev
  Red Hat based sudo yum install make gcc gcc-c++ kernel-devel cmake git libv4l-devel
  Arch Linux    sudo pacman -S base-devel cmake git v4l-utils
  ============= =====================================================================

.. ::

  `Installation of System Dependencies <https://github.com/LuaDist/Repository/wiki/Installation-of-System-Dependencies>`_

Getting Source Code
--------------------

.. code-block:: bash

  sudo apt-get install git
  git clone https://github.com/slightech/MYNT-EYE-S-SDK.git

Required Packages
------------------

.. code-block:: bash

  cd <sdk>
  make init

* `OpenCV <https://opencv.org/>`_

.. tip::

  To build and install Opencv(Not support 4.0+), Please refer to `Installation in Linux <https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html>`_ . Alternatively, refer to the command below:

  .. code-block:: bash

    [compiler] sudo apt-get install build-essential
    [required] sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
    [optional] sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

    $ git clone https://github.com/opencv/opencv.git
    $ cd opencv/
    $ git checkout tags/3.4.1

    $ mkdir _build
    $ cd _build/

    $ cmake \
    -DCMAKE_BUILD_TYPE=RELEASE \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    \
    -DWITH_CUDA=OFF \
    \
    -DBUILD_DOCS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    ..

    $ make -j4
    $ sudo make install

Building code
--------------

.. tip::

  If opencv is installed in custom directory or if you want to specify a version, you should set the path before building:

  .. code-block:: bash

    # OpenCV_DIR is the directory where your OpenCVConfig.cmake exists
    export OpenCV_DIR=~/opencv

  Otherwise, CMake will prompt cannot find OpenCV. If you need sdk without OpenCV, please read :ref:`sdk_without_opencv` .

Build and install:

.. code-block:: bash

  cd <sdk>
  make install

Finally, sdk will install in ``/usr/local`` by default.

Building samples
----------------

.. code-block:: bash

  cd <sdk>
  make samples

Run samples:

.. code-block:: bash

  ./samples/_output/bin/api/camera_a

Tutorial samples, please read :ref:`data` and :ref:`ctrl` .

Building tools
---------------

.. code-block:: bash

  cd <sdk>
  make tools

Installation requirement:

.. code-block:: bash

  cd <sdk>/tools/
  sudo pip install -r requirements.txt

The usage of tools and scripts will be introduced later.

Conclusion
-----------

If your project will use SDK, you can refer to the settings in ``samples/CMakeLists.txt`` for CMake. Alternatively, import the head file and dynamic library in the installation directory.
