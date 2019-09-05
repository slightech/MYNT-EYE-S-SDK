.. _data_write_img_params:

Write Image Parameters
=======================

The SDK provides a tool ``write_img_params`` for writing image parameters.

For getting image parameters, please read :ref:`get_img_params`. This is used to calculate the deviation.

Reference commands:

.. code-block:: bash

  ./samples/_output/bin/write_img_params samples/config/img.params

  # Windows
  .\samples\_output\bin\write_img_params.bat samples\config\img.params

.. warning::

  Please don't override parameters, you can use ``save_all_infos`` to backup parameters.

And, `samples/config/S1030/img.params.pinhole <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/config/S1030/img.params.pinhole>`_ is the path of S1030 pihole parameters file. If you calibrated parameters yourself, you can edit it and run previous commands to write them into the devices.

.. tip::

  The image calibration parameters of S21XX are in  ``samples/config/S21XX``
  The image calibration parameters of S1030 are in   ``samples/config/S1030``

.. tip::

  You can also write into devices with ``SN*.conf`` provided by old SDK.

