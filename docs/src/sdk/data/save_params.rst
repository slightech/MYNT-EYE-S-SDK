.. _data_save_params:

Save Device Infomation And Parameters
=====================================

The SDK provides a tool ``save_all_infos`` for save information and parameters.

Reference commands:

.. code-block:: bash

  ./samples/_output/bin/save_all_infos

  # Windows
  .\samples\_output\bin\save_all_infos.bat

Reference result on Linux:

.. code-block:: bash

  $ ./samples/_output/bin/save_all_infos
  I0512 21:40:08.687088  4092 utils.cc:26] Detecting MYNT EYE devices
  I0512 21:40:09.366693  4092 utils.cc:33] MYNT EYE devices:
  I0512 21:40:09.366734  4092 utils.cc:37]   index: 0, name: MYNT-EYE-S1000
  I0512 21:40:09.366757  4092 utils.cc:43] Only one MYNT EYE device, select index: 0
  I0512 21:40:09.367609  4092 save_all_infos.cc:38] Save all infos to "config/SN0610243700090720"

Result save into ``<workdir>/config`` by default. You can also add parameters to select other directory for save.

Saved contents:

.. code-block:: none

  <workdir>/
  └─config/
     └─SN0610243700090720/
        ├─device.info
        ├─img.params
        └─imu.params
