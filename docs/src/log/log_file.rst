.. _log_file:

Enable log file
================

.. tip::

  If import glog to build.

The general configuration of the log in the head file `logger.h <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/include/mynteye/logger.h>`_ .

Uncomment ``FLAGS_log_dir = \".\";`` recompile and save to current work directory. Run ``camera_a`` log file as follows:

.. code-block:: none

  <workdir>/
  ├─camera_a.ERROR
  ├─camera_a.FATAL
  ├─camera_a.INFO
  ├─camera_a.WARNING
  ├─camera_a.john-ubuntu.john.log.ERROR.20180513-141833.519
  ├─camera_a.john-ubuntu.john.log.FATAL.20180513-141833.519
  ├─camera_a.john-ubuntu.john.log.INFO.20180513-141832.519
  └─camera_a.john-ubuntu.john.log.WARNING.20180513-141833.519

``camera_a.INFO`` shows the program and levers of log it is running. The link to the real log file is ``camera_a.john-ubuntu.john.log.INFO.20180513-141832.519``. Even if it ran several times, ``camera_a.INFO`` still leaves the link to last log file.

Excute `make cleanlog` to clean all log files.
