.. _sdk_install_windows_src:

Windows Source Installation
================================

.. only:: html

  +-----------------+
  | Windows 10      |
  +=================+
  | |build_passing| |
  +-----------------+

  .. |build_passing| image:: https://img.shields.io/badge/build-passing-brightgreen.svg?style=flat

.. only:: latex

  +-----------------+
  | Windows 10      |
  +=================+
  | ✓               |
  +-----------------+

.. tip::

  Windows does not provide Visual Studio ``*.sln`` file directly and requires CMake to build. Firstly, CMake can be used across multiple platforms, it is easy to configure and can be maintained in a sustainable way. Secondly, the third-party codes (Glog, OpenCV) are built using CMake.

.. tip::

  There is currently no binary installer available, which requires you to compile from source. It is also the process of configuring the development environment.

Prerequisites
-------------

CMake (provide build）
~~~~~~~~~~~~~~~~~~~~~~

* `CMake <https://cmake.org/download/>`_，used to build and compile (necessary).
* `Git <https://git-scm.com/downloads>`_， used to get code (optional).
* `Doxygen <http://www.stack.nl/~dimitri/doxygen/download.html>`_， used to generate documents (optional).

After you install the above tools, confirm that you can run this command in CMD (Command Prompt):

.. code-block:: bat

  >cmake --version
  cmake version 3.10.1

  >git --version
  git version 2.11.1.windows.1

  >doxygen --version
  1.8.13

Visual Studio (provide compilation)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* `Visual Studio <https://www.visualstudio.com/>`_

  * `Visual Studio 2017 <https://my.visualstudio.com/Downloads?q=Visual Studio 2017>`_
  * `Visual Studio 2015 <https://my.visualstudio.com/Downloads?q=Visual Studio 2015>`_

* `Windows 10 SDK <https://developer.microsoft.com/en-US/windows/downloads/windows-10-sdk>`_

After installing Visual Studio, confirm that the following command can run in the Visual Studio Command Prompt:

.. code-block:: bat

  >cl
  Microsoft (R) C/C++ Optimizing Compiler Version 19.14.26429.4 for x86

  >msbuild
  Microsoft (R) 生成引擎版本 15.7.179.6572

.. tip::

  Visual Studio Command Prompt can be opened from the Start menu,

  .. image:: ../../images/sdk/vs_cmd_menu.png
    :width: 30%

  You can also open it from the Visual Studio Tools menu.

  .. image:: ../../images/sdk/vs_cmd.png
    :width: 40%

  However, if you do not have the Visual Studio 2015 Tools menu, you can add one yourself.

  Open Tools's External Tools... and Add the following:

  ================= =======================================================================================
  Field             Value
  ================= =======================================================================================
  Title             Visual Studio Command Prompt
  Command           ``C:\Windows\System32\cmd.exe``
  Arguments         ``/k "C:\Program Files (x86)\Microsoft Visual Studio 14.0\Common7\Tools\VsDevCmd.bat"``
  Initial Directory ``$(SolutionDir)``
  ================= =======================================================================================

  In Visual Studio command Prompt, you can use the compile command ``cl`` ``link`` ``lib`` ``msbuild``, etc.(need finish ``MSYS2``and ``Getting Source Code`` steps first)

  .. image:: ../../images/sdk/vs_cmd_test.png

MSYS2 (provide Linux command)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* `MSYS2 <http://www.msys2.org/>`_

  * `mirror <https://lug.ustc.edu.cn/wiki/mirrors/help/msys2>`_
  * `pacman <https://wiki.archlinux.org/index.php/pacman>`_

After installation, verify that the following path has been added to the system environment variable PATH:

.. code-block:: none

    C:\msys64\usr\bin

Then, open MSYS2 MSYS, perform the update and install ``make``:

.. code-block:: bash

  $ pacman -Syu
  $ pacman -S make

Finally, the CMD (Command Prompt) can run the following command:

.. code-block:: bat

  >make --version
  GNU Make 4.2.1

Getting Source Code
--------------------

.. code-block:: bat

  git clone https://github.com/slightech/MYNT-EYE-S-SDK.git

Required Packages
-----------------

.. code-block:: bat

  >cd <sdk>
  >make init
  Make init
  Init deps
  Install cmd: pacman -S
  Install deps: git clang-format
  pacman -S clang-format (not exists)
  error: target not found: clang-format
  pip install --upgrade autopep8 cpplint pylint requests
  ...
  Init git hooks
  ERROR: clang-format-diff is not installed!
  Expect cmake version >= 3.0
  cmake version 3.10.1

* `OpenCV <https://opencv.org/>`_

.. tip::

  The official OpenCV provides the ``exe`` for installation. If you want to compile from the source code, see the Official document `Installation in Windows <https://docs.opencv.org/master/d3/d52/tutorial_windows_install.html>`_ . or refer to the following command:

  .. code-block:: bat

    >git clone https://github.com/opencv/opencv.git
    >cd opencv
    >git checkout tags/3.4.1

    >cd opencv
    >mkdir _build
    >cd _build

    >cmake ^
    -D CMAKE_BUILD_TYPE=RELEASE ^
    -D CMAKE_INSTALL_PREFIX=C:/opencv ^
    -D WITH_CUDA=OFF ^
    -D BUILD_DOCS=OFF ^
    -D BUILD_EXAMPLES=OFF ^
    -D BUILD_TESTS=OFF ^
    -D BUILD_PERF_TESTS=OFF ^
    -G "Visual Studio 15 2017 Win64" ^
    ..

    >msbuild ALL_BUILD.vcxproj /property:Configuration=Release
    >msbuild INSTALL.vcxproj /property:Configuration=Release

Building Code
--------------

.. tip::

  If OpenCV is installed in a custom directory or wants to specify a version, you can set the path as follows before compiling:

  .. code-block:: bat

    # OpenCV_DIR is hte path where OpenCVConfig.cmake in
    set OpenCV_DIR=C:\opencv

  Otherwise, CMake will prompt that OpenCV could not be found. If you don't want to rely on OpenCV, read :ref:`sdk_without_opencv` .

Build and install:

.. code-block:: bat

  cd <sdk>
  make install

Finally, the SDK will install in ``<sdk>/_install`` by default.

Building samples
----------------

.. code-block:: bat

  cd <sdk>
  make samples

Run samples:

.. code-block:: bat

  .\samples\_output\bin\api\camera_a.bat

For tutorial samples, please read :ref:`data` and :ref:`ctrl` .

.. tip::

  All compiled sample programs ``exe`` will have a corresponding ``bat``. ``bat`` will temporarily set system environment variables and then run ``exe``. So it is recommended to run ``bat``.

  If you run``exe`` directly, it may prompt that cannot find ``dll``. Then you should add ``<sdk>\\_install\\bin`` ``%OPENCV_DIR%\\bin`` to ``PATH`` in system environment variable.

  How to set the environment variable for OpenCV, refer to the official document `Set the OpenCV environment variable and add it to the systems path <https://docs.opencv.org/master/d3/d52/tutorial_windows_install.html#tutorial_windows_install_path>`_ .

Building tools
---------------

.. code-block:: bat

  cd <sdk>
  make tools

The usage of tools and scripts will be introduced later.

.. tip::

  The script is based on Python. You need to install Python and its package management tool pip first, and then install the dependencies as follows:

  .. code-block:: bat

    cd <sdk>\tools
    pip install -r requirements.txt

  Note: Python is also in MSYS2, but fail install Matplotlib in test.

Conclusion
-----------

If your project will use SDK, you can refer to the settings in ``samples/CMakeLists.txt`` for CMake. Or just import the head file and dynamic library in the installation directory.
