.. _data_write_imu_params:

Write IMU Parameters
=====================

SDK provides the tool ``write_imu_params`` to write IMU parameters.

Information about how to get IMU parameters, please read :ref:`get_imu_params` .

Reference commands:

.. code-block:: bash

  ./samples/_output/bin/write_imu_params samples/config/imu.params

  # Windows
  .\samples\_output\bin\write_imu_params.bat samples\config\imu.params

The path of parameters folder can be found in `samples/config/imu.params <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/config>`_ . If you calibrated the parameters yourself, you can edit the file and run above commands to write them into the device.

.. warning::

  Please don't override parameters, you can use ``save_all_infos`` to backup parameters.