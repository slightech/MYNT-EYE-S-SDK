.. _framerate:

Set the frame rate of image & IMU frequency
============================================

Using the ``SetOptionValue()`` function in the API, you can set various control values​for the current device.

For mynteye s1030, to set the image frame rate and IMU frequency, set ``Option::FRAME_RATE`` and ``Option::IMU_FREQUENCY``.

.. Attention::

  * The effective fps of the image: 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60.
  * The effective frequency of IMU: 100, 200, 250, 333, 500.

For mynteye s21XX, the image frame rate should be selected when running the sample, and the frame rate and resolution are combined as follows:

.. code-block:: bash

  index: 0, request: width: 1280, height: 400, format: Format::BGR888, fps: 10
  index: 1, request: width: 1280, height: 400, format: Format::BGR888, fps: 20
  index: 2, request: width: 1280, height: 400, format: Format::BGR888, fps: 30
  index: 3, request: width: 1280, height: 400, format: Format::BGR888, fps: 60
  index: 4, request: width: 2560, height: 800, format: Format::BGR888, fps: 10
  index: 5, request: width: 2560, height: 800, format: Format::BGR888, fps: 20
  index: 6, request: width: 2560, height: 800, format: Format::BGR888, fps: 30


Reference Code:

s1030：

.. code-block:: c++

  auto &&api = API::Create(argc, argv);

  // Attention: must set FRAME_RATE and IMU_FREQUENCY together, otherwise won't
  // succeed.

  // FRAME_RATE values: 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60
  api->SetOptionValue(Option::FRAME_RATE, 25);
  // IMU_FREQUENCY values: 100, 200, 250, 333, 500
  api->SetOptionValue(Option::IMU_FREQUENCY, 500);

  LOG(INFO) << "Set FRAME_RATE to " << api->GetOptionValue(Option::FRAME_RATE);
  LOG(INFO) << "Set IMU_FREQUENCY to "
            << api->GetOptionValue(Option::IMU_FREQUENCY);

s21XX：

.. code-block:: c++

  auto &&api = API::Create(argc, argv);
  if (!api) return 1;
  
  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);

  LOG(INFO) << "Please set frame rate by 'SelectStreamRequest()'";



Reference running results on Linux:

s1030：

.. code-block:: bash

  $ ./samples/_output/bin/ctrl_framerate
  I0513 14:05:57.218222 31813 utils.cc:26] Detecting MYNT EYE devices
  I0513 14:05:57.899404 31813 utils.cc:33] MYNT EYE devices:
  I0513 14:05:57.899430 31813 utils.cc:37]   index: 0, name: MYNT-EYE-S1000
  I0513 14:05:57.899435 31813 utils.cc:43] Only one MYNT EYE device, select index: 0
  I0513 14:05:58.076257 31813 framerate.cc:36] Set FRAME_RATE to 25
  I0513 14:05:58.076836 31813 framerate.cc:37] Set IMU_FREQUENCY to 500
  I0513 14:06:21.702361 31813 framerate.cc:82] Time beg: 2018-05-13 14:05:58.384967, end: 2018-05-13 14:06:21.666115, cost: 23281.1ms
  I0513 14:06:21.702388 31813 framerate.cc:85] Img count: 573, fps: 24.6122
  I0513 14:06:21.702404 31813 framerate.cc:87] Imu count: 11509, hz: 494.348

s21XX：

.. code-block:: bash

  $ ./samples/_output/bin/ctrl_framerate 
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
  2
  I/framerate.cc:54 Please set frame rate by 'SelectStreamRequest()'
  I/framerate.cc:99 Time beg: 2018-12-29 10:05:08.203095, end: 2018-12-29 10:08:20.074969, cost: 191872ms
  I/framerate.cc:102 Img count: 5759, fps: 30.0148
  I/framerate.cc:104 Imu count: 77163, hz: 402.159


After the sample program finishes running with ``ESC/Q``, it will output the calculated value of the frame rate of image & IMU frequency.

Complete code samples，please see `ctrl_framerate.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/ctrl_framerate.cc>`_ .
