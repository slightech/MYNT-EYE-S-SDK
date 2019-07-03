.. _sdk_install_windows_exe:

Windows EXE Installation
=============================

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

Download and install SDK
-------------------------

.. tip::

  Download here: mynteye-s-x.x.x-win-x64-opencv-3.4.3.exe `Google Drive <https://drive.google.com/open?id=1PYC_5Mh2pzLFVXkYlkllEzPnr50EbKht>`_ `Baidu Pan(key:rj4k) <https://pan.baidu.com/s/1yCKjvivB2gsqTV8xyY7DQg>`_ .

After you install the win pack of SDK, there will be a shortcut to the SDK root directory on your desktop.

Goto the ``<SDK_ROOT_DIR>\\bin\\samples\\tutorials`` directory and click ``get_stereo.exe`` to run.

Generate samples project
-------------------------

First, you should install `Visual Studio 2017 <https://visualstudio.microsoft.com/>`_ and `CMake <https://cmake.org/>`_ .

Second, goto the ``<SDK_ROOT_DIR>\\samples`` directory and click ``generate.bat`` to generate project.

.. tip::

  Right click sample and select ``Set as StartUp Project``，then launch with Release x64 mode.

The tutorials of samples are here: https://slightech.github.io/MYNT-EYE-S-SDK-Guide/src/data/contents.html.

Start using SDK with Visual Studio 2017
----------------------------------------

Goto the ``<SDK_ROOT_DIR>\\projects\\vs2017``, see the ``README.md``.
