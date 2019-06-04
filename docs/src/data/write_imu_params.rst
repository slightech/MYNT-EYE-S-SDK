.. _data_write_imu_params:

Write IMU parameters
=====================

SDK provides the tool ``imu_params_writer`` to write IMU parameters. For deltail, please read `tools/README.md <https://github.com/slightech/MYNT-EYE-S-SDK/tree/master/tools>`_ .

Information about how to get IMU parameters, please read :ref:`get_imu_params` .

Reference commands:

.. code-block:: bash

  ./tools/_output/bin/writer/imu_params_writer tools/writer/config/imu.params

  # Windows
  .\tools\_output\bin\writer\imu_params_writer.bat tools\writer\config\imu.params

The path of parameters file can be found in `tools/writer/config/img.params <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/tools/writer/config/img.params>`_ . If you calibrated the parameters yourself, you can edit the file and run above commands to write them into the device.

.. warning::

  Please don't override parameters, you can use ``save_all_infos`` to backup parameters.