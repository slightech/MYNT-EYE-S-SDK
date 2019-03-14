.. _dataset:

Recording data sets
====================

The SDK provides the tool ``record`` for recording data sets. Tool details can be seen in `tools/README.md <https://github.com/slightech/MYNT-EYE-S-SDK/tree/master/tools>`_ .

Reference run command:

.. code-block:: bash

  ./tools/_output/bin/dataset/record

  # Windows
  .\tools\_output\bin\dataset\record.bat

Reference run results on Linux:

.. code-block:: bash

  $ ./tools/_output/bin/dataset/record
  I0513 21:28:57.128947 11487 utils.cc:26] Detecting MYNT EYE devices
  I0513 21:28:57.807116 11487 utils.cc:33] MYNT EYE devices:
  I0513 21:28:57.807155 11487 utils.cc:37]   index: 0, name: MYNT-EYE-S1000
  I0513 21:28:57.807163 11487 utils.cc:43] Only one MYNT EYE device, select index: 0
  I0513 21:28:57.808437 11487 channels.cc:114] Option::GAIN: min=0, max=48, def=24, cur=24
  I0513 21:28:57.809999 11487 channels.cc:114] Option::BRIGHTNESS: min=0, max=240, def=120, cur=120
  I0513 21:28:57.818678 11487 channels.cc:114] Option::CONTRAST: min=0, max=255, def=127, cur=127
  I0513 21:28:57.831529 11487 channels.cc:114] Option::FRAME_RATE: min=10, max=60, def=25, cur=25
  I0513 21:28:57.848914 11487 channels.cc:114] Option::IMU_FREQUENCY: min=100, max=500, def=200, cur=500
  I0513 21:28:57.865185 11487 channels.cc:114] Option::EXPOSURE_MODE: min=0, max=1, def=0, cur=0
  I0513 21:28:57.881434 11487 channels.cc:114] Option::MAX_GAIN: min=0, max=48, def=48, cur=48
  I0513 21:28:57.897598 11487 channels.cc:114] Option::MAX_EXPOSURE_TIME: min=0, max=240, def=240, cur=240
  I0513 21:28:57.913918 11487 channels.cc:114] Option::DESIRED_BRIGHTNESS: min=0, max=255, def=192, cur=192
  I0513 21:28:57.930177 11487 channels.cc:114] Option::IR_CONTROL: min=0, max=160, def=0, cur=0
  I0513 21:28:57.946341 11487 channels.cc:114] Option::HDR_MODE: min=0, max=1, def=0, cur=0
  Saved 1007 imgs, 20040 imus to ./dataset
  I0513 21:29:38.608772 11487 record.cc:118] Time beg: 2018-05-13 21:28:58.255395, end: 2018-05-13 21:29:38.578696, cost: 40323.3ms
  I0513 21:29:38.608853 11487 record.cc:121] Img count: 1007, fps: 24.9732
  I0513 21:29:38.608873 11487 record.cc:123] Imu count: 20040, hz: 496.983

Results save into ``<workdir>/dataset`` by default. You can also add parameter, select other directory to save.

Record contents:

.. code-block:: none

  <workdir>/
  └─dataset/
     ├─left/
     │  ├─stream.txt  # Image infomation
     │  ├─000000.png  # Image，index 0
     │  └─...
     ├─right/
     │  ├─stream.txt  # Image information
     │  ├─000000.png  # Image，index 0
     │  └─...
     └─motion.txt  # IMU information
