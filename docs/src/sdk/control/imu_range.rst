.. _imu_range:

Set the range of accelerometer & gyroscope
==========================================

Using the ``SetOptionValue()`` function in the API, you can set various control values for the current device.

To set the range of accelerometer and gyroscope, set ``Option::ACCELEROMETER_RANGE`` and ``Option::GYROSCOPE_RANGE``.

.. Attention::
  For mynteye s1030, the available settings are:
  
  * The effective range of accelerometer(unit:g): 4, 8, 16, 32.
  * Gyroscope Range Valid value (unit: DEG/S): 500, 1000, 2000, 4000.

  For mynteye s21XX, the available settings are:
  
  * The effective range of accelerometer(unit:g): 6, 12, 24, 48.
  * The effective range of gyroscope(unit:deg/s): 250, 500, 1000, 2000, 4000.

Reference Code:

s1030：

.. code-block:: c++

  auto &&api = API::Create(argc, argv);
  if (!api)
    return 1;

  // ACCELEROMETER_RANGE values: 4, 8, 16, 32
  api->SetOptionValue(Option::ACCELEROMETER_RANGE, 8);
  // GYROSCOPE_RANGE values: 500, 1000, 2000, 4000
  api->SetOptionValue(Option::GYROSCOPE_RANGE, 1000);

  LOG(INFO) << "Set ACCELEROMETER_RANGE to "
            << api->GetOptionValue(Option::ACCELEROMETER_RANGE);
  LOG(INFO) << "Set GYROSCOPE_RANGE to "
            << api->GetOptionValue(Option::GYROSCOPE_RANGE);

s21XX：

.. code-block:: c++

  auto &&api = API::Create(argc, argv);
  if (!api) return 1;

  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);

  // ACCELEROMETER_RANGE values: 6, 12, 24, 48
  api->SetOptionValue(Option::ACCELEROMETER_RANGE, 6);
  // GYROSCOPE_RANGE values: 250, 500, 1000, 2000, 4000
  api->SetOptionValue(Option::GYROSCOPE_RANGE, 1000);

  LOG(INFO) << "Set ACCELEROMETER_RANGE to "
            << api->GetOptionValue(Option::ACCELEROMETER_RANGE);
  LOG(INFO) << "Set GYROSCOPE_RANGE to "
            << api->GetOptionValue(Option::GYROSCOPE_RANGE);


Reference running results on Linux:

s1030：

.. code-block:: bash

  $ ./samples/_output/bin/ctrl_imu_range
  I/utils.cc:28 Detecting MYNT EYE devices
  I/utils.cc:38 MYNT EYE devices:
  I/utils.cc:41   index: 0, name: MYNT-EYE-S1030, sn: 4B4C1F1100090712
  I/utils.cc:49 Only one MYNT EYE device, select index: 0
  I/imu_range.cc:34 Set ACCELEROMETER_RANGE to 8
  I/imu_range.cc:36 Set GYROSCOPE_RANGE to 1000
  I/imu_range.cc:81 Time beg: 2018-11-21 15:34:57.726428, end: 2018-11-21 15:35:12.190478, cost: 14464ms
  I/imu_range.cc:84 Img count: 363, fps: 25.0967
  I/imu_range.cc:86 Imu count: 2825, hz: 195.312

s21XX：

.. code-block:: bash

  $ ./samples/_output/bin/ctrl_imu_range 
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
  3
  I/imu_range.cc:51 Set ACCELEROMETER_RANGE to 6
  I/imu_range.cc:53 Set GYROSCOPE_RANGE to 1000
  I/imu_range.cc:98 Time beg: 2018-12-29 10:03:10.706211, end: 2018-12-29 10:04:12.497427, cost: 61791.2ms
  I/imu_range.cc:101 Img count: 3706, fps: 59.9762
  I/imu_range.cc:103 Imu count: 24873, hz: 402.533

After the sample program finishes running with ``ESC/Q``, the ranges of imu setting is complete. The ranges will be kept inside the hardware and not affected by power off.

Complete code samples，please see `ctrl_imu_range.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/ctrl_imu_range.cc>`_.
