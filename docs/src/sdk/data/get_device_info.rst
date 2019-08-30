.. _data_get_device_info:

Get Device Information
======================

Use ``GetInfo()`` function to get various current information values.

Reference code snippet:

.. code-block:: c++

  auto &&api = API::Create(argc, argv);

  LOG(INFO) << "Device name: " << api->GetInfo(Info::DEVICE_NAME);
  LOG(INFO) << "Serial number: " << api->GetInfo(Info::SERIAL_NUMBER);
  LOG(INFO) << "Firmware version: " << api->GetInfo(Info::FIRMWARE_VERSION);
  LOG(INFO) << "Hardware version: " << api->GetInfo(Info::HARDWARE_VERSION);
  LOG(INFO) << "Spec version: " << api->GetInfo(Info::SPEC_VERSION);
  LOG(INFO) << "Lens type: " << api->GetInfo(Info::LENS_TYPE);
  LOG(INFO) << "IMU type: " << api->GetInfo(Info::IMU_TYPE);
  LOG(INFO) << "Nominal baseline: " << api->GetInfo(Info::NOMINAL_BASELINE);

Reference result on Linux:

.. code-block:: bash

  $ ./samples/_output/bin/get_device_info
  I0503 16:40:21.109391 32106 utils.cc:13] Detecting MYNT EYE devices
  I0503 16:40:21.604116 32106 utils.cc:20] MYNT EYE devices:
  I0503 16:40:21.604127 32106 utils.cc:24]   index: 0, name: MYNT-EYE-S1000
  I0503 16:40:21.604142 32106 utils.cc:30] Only one MYNT EYE device, select index: 0
  I0503 16:40:21.615054 32106 get_device_info.cc:10] Device name: MYNT-EYE-S1000
  I0503 16:40:21.615113 32106 get_device_info.cc:11] Serial number: 0610243700090720
  I0503 16:40:21.615129 32106 get_device_info.cc:12] Firmware version: 2.0
  I0503 16:40:21.615139 32106 get_device_info.cc:13] Hardware version: 2.0
  I0503 16:40:21.615146 32106 get_device_info.cc:14] Spec version: 1.0
  I0503 16:40:21.615155 32106 get_device_info.cc:15] Lens type: 0000
  I0503 16:40:21.615164 32106 get_device_info.cc:16] IMU type: 0000
  I0503 16:40:21.615171 32106 get_device_info.cc:17] Nominal baseline: 120

Complete code examples, see `get_device_info.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/get_device_info.cc>`_ .
