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

And, `tools/writer/config/img.params <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/tools/writer/config/img.params>`_ is the path of parameters file. If you calibrated parameters yourself, you can edit it and run previous commands to write them into the devices.

.. tip::

  You can also write into devices with ``SN*.conf`` provided by old SDK.

.. warning::

  Please don't override parameters, you can use ``save_all_infos`` to backup parameters.