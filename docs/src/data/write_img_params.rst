.. _write_img_params:

Write image parameters
=======================

The SDK provides a tool ``img_params_writer`` for writing image parameters. For details, read `tools/README.md <https://github.com/slightech/MYNT-EYE-S-SDK/tree/master/tools>`_ .

For getting image parameters, please read :ref:`get_img_params`. This is used to calculate the deviation.

Reference commands:

.. code-block:: bash

  ./tools/_output/bin/writer/img_params_writer tools/writer/config/img.params

  # Windows
  .\tools\_output\bin\writer\img_params_writer.bat tools\writer\config\img.params

.. warning::

  Please don't override parameters, you can use ``save_all_infos`` to backup parameters.

And, `tools/writer/config/S1030/img.params.pinhole <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/tools/writer/config/S1030/img.params.pinhole>`_ is the path of S1030 pihole parameters file. If you calibrated parameters yourself, you can edit it and run previous commands to write them into the devices.

.. tip::

  The image calibration parameters of S2100/S210A are in  ``tools/writer/config/S210A``
  The image calibration parameters of S1030 are in   ``tools/writer/config/S1030``

.. tip::

  You can also write into devices with ``SN*.conf`` provided by old SDK.

