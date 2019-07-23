.. _sdk_without_opencv:

OpenCV Description
===================

SDK provides a three-tier interface with OpenCV dependencies:

* ``api``, upper interface, with OpenCV dependencies
* ``device``, interlayer interface, without OpenCV dependencies
* ``uvc``, bottom interface, without OpenCV dependencies

If you don't want to use OpenCV, edit ``<sdk>/cmake/Option.cmake`` , set ``WITH_API`` to ``OFF``. This will stop the compilation of the interface api:

.. code-block:: cmake

  option(WITH_API "Build with API layer, need OpenCV" ON)

For samples for the interface ``device``, please refer to `device/camera.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/device/camera.cc>`_ .
