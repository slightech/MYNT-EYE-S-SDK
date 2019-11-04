.. _manual_exposure:

Enable manual exposure and its adjustment function
===================================================

Using the ``SetOptionValue()`` function of the API, you can set various control values for the current open device.

To enabling manual exposure, set ``Option::EXPOSURE_MODE`` to ``1``.

For mynteye s1030, during manual exposure, the settings available for adjustment are:

* ``Option::GAIN`` Gain.
* ``Option::BRIGHTNESS`` Brightness (Exposure time).
* ``Option::CONTRAST`` Contrast (Black level calibration).

For mynteye s21XX, during manual exposure, the settings available for adjustment are:

* ``Option::BRIGHTNESS`` Brightness (Exposure time).


Reference Code:

s1030：

.. code-block:: c++

  auto &&api = API::Create(argc, argv);

  // manual-exposure: 1
  api->SetOptionValue(Option::EXPOSURE_MODE, 1);

  // gain: range [0,48], default 24
  api->SetOptionValue(Option::GAIN, 24);
  // brightness/exposure_time: range [0,240], default 120
  api->SetOptionValue(Option::BRIGHTNESS, 120);
  // contrast/black_level_calibration: range [0,254], default 116
  api->SetOptionValue(Option::CONTRAST, 116);

  LOG(INFO) << "Enable manual-exposure";
  LOG(INFO) << "Set GAIN to " << api->GetOptionValue(Option::GAIN);
  LOG(INFO) << "Set BRIGHTNESS to " << api->GetOptionValue(Option::BRIGHTNESS);
  LOG(INFO) << "Set CONTRAST to " << api->GetOptionValue(Option::CONTRAST);

s21XX：

.. code-block:: c++

  auto &&api = API::Create(argc, argv);

  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);

  // manual-exposure: 1
  api->SetOptionValue(Option::EXPOSURE_MODE, 1);

  // brightness/exposure_time: range [1,255], default 70
  api->SetOptionValue(Option::BRIGHTNESS, 70);

  LOG(INFO) << "Enable manual-exposure";
  LOG(INFO) << "Set EXPOSURE_MODE to "
            << api->GetOptionValue(Option::EXPOSURE_MODE);
  LOG(INFO) << "Set BRIGHTNESS to "
            << api->GetOptionValue(Option::BRIGHTNESS);


Reference running results on Linux:

s1030：

.. code-block:: bash

  $ ./samples/_output/bin/ctrl_manual_exposure
  I0513 14:09:17.104431 31908 utils.cc:26] Detecting MYNT EYE devices
  I0513 14:09:17.501519 31908 utils.cc:33] MYNT EYE devices:
  I0513 14:09:17.501551 31908 utils.cc:37]   index: 0, name: MYNT-EYE-S1000
  I0513 14:09:17.501562 31908 utils.cc:43] Only one MYNT EYE device, select index: 0
  I0513 14:09:17.552918 31908 manual_exposure.cc:37] Enable manual-exposure
  I0513 14:09:17.552953 31908 manual_exposure.cc:38] Set GAIN to 24
  I0513 14:09:17.552958 31908 manual_exposure.cc:39] Set BRIGHTNESS to 120
  I0513 14:09:17.552963 31908 manual_exposure.cc:40] Set CONTRAST to 116

s21XX：

.. code-block:: bash

  $ ./samples/_output/bin/ctrl_manual_exposure 
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
  I/manual_exposure.cc:62 Enable manual-exposure
  I/manual_exposure.cc:63 Set EXPOSURE_MODE to 1
  I/manual_exposure.cc:65 Set BRIGHTNESS to 70


The sample program displays an image with a real exposure time in the upper left corner, in milliseconds.

Complete code samples，see `ctrl_manual_exposure.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/ctrl_manual_exposure.cc>`_ .
