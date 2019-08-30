.. _imu_low_pass_filter:

Low-pass Filter
===============

Using the ``SetOptionValue()`` function in the API, you can set various control values ​​for the current device.

To set the value of accelerometer low-pass filter and gyroscope low-pass filter, set ``Option::ACCELEROMETER_LOW_PASS_FILTER`` and ``Option::GYROSCOPE_LOW_PASS_FILTER``.

.. Attention::
  * s1030 doesn't support this feature

Reference Code:

.. code-block:: c++

  auto &&api = API::Create(argc, argv);
  if (!api) return 1;

  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);

  // ACCELEROMETER_RANGE values: 0, 1, 2
  api->SetOptionValue(Option::ACCELEROMETER_LOW_PASS_FILTER, 2);
  // GYROSCOPE_RANGE values: 23, 64
  api->SetOptionValue(Option::GYROSCOPE_LOW_PASS_FILTER, 64);

  LOG(INFO) << "Set ACCELEROMETER_LOW_PASS_FILTER to "
            << api->GetOptionValue(Option::ACCELEROMETER_LOW_PASS_FILTER);
  LOG(INFO) << "Set GYROSCOPE_LOW_PASS_FILTER to "
            << api->GetOptionValue(Option::GYROSCOPE_LOW_PASS_FILTER);


Reference running results on Linux:

.. code-block:: bash

  $ ./samples/_output/bin/ctrl_imu_low_pass_filter 
  I/utils.cc:30 Detecting MYNT EYE devices
  I/utils.cc:40 MYNT EYE devices:
  I/utils.cc:43   index: 0, name: MYNT-EYE-S210A, sn: 07C41A190009071F
  I/utils.cc:51 Only one MYNT EYE device, select index: 0
  I/utils.cc:79 MYNT EYE devices:
  I/utils.cc:82   index: 0, request: width: 1280, height: 400, format: Format::BGR888, fps: 10
  I/utils.cc:82   index: 1, request: width: 1280, height: 400, format: Format::BGR888, fps: 20
  I/utils.cc:82   index: 2, request: width: 1280, height: 400, format: Format::BGR888, fps: 30
  I/utils.cc:82   index: 3, request: width: 1280, height: 400, format: Format::BGR888, fps: 60
  I/utils.cc:82   index: 4, request: width: 2560, height: 800, format: Format::BGR888, fps: 10
  I/utils.cc:82   index: 5, request: width: 2560, height: 800, format: Format::BGR888, fps: 20
  I/utils.cc:82   index: 6, request: width: 2560, height: 800, format: Format::BGR888, fps: 30
  I/utils.cc:93 There are 7 stream requests, select index: 
  1
  I/imu_low_pass_filter.cc:48 Set ACCELEROMETER_LOW_PASS_FILTER to 2
  I/imu_low_pass_filter.cc:50 Set GYROSCOPE_LOW_PASS_FILTER to 64
  I/imu_low_pass_filter.cc:96 Time beg: 2018-12-29 13:53:42.296299, end: 2018-12-29 14:06:33.295960, cost: 771000ms
  I/imu_low_pass_filter.cc:99 Img count: 15412, fps: 19.9896
  I/imu_low_pass_filter.cc:101 Imu count: 309891, hz: 401.934

After the sample program finishes running with ``ESC/Q``, the low-pass filter of imu setting is complete. The ranges will be kept inside the hardware and not affected by power off.

Complete code samples，please see `ctrl_imu_low_pass_filter.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/ctrl_imu_low_pass_filter.cc>`_ 。
