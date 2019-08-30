.. _data_get_img_params:

Get Image Calibration Parameters
================================

Use ``GetIntrinsics()`` & ``GetExtrinsics()`` to get image calibration parameters.

.. tip::
  The detailed meaning of parameters can reference the files in ``tools/writer/config`` , of these
  the image calibration parameters of S2100/S210A are in  ``tools/writer/config/S210A``
  the image calibration parameters of S1030 are in   ``tools/writer/config/S1030``

Note

Camera Intrinsics/Extrinsics, please ref to: ros `CameraInfo <http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html>`_.

Reference code snippet:

.. code-block:: c++

  auto &&api = API::Create(argc, argv);

  LOG(INFO) << "Intrinsics left: {" << *api->GetIntrinsicsBase(Stream::LEFT)
            << "}";
  LOG(INFO) << "Intrinsics right: {" << *api->GetIntrinsicsBase(Stream::RIGHT)
            << "}";
  LOG(INFO) << "Extrinsics right to left: {"
            << api->GetExtrinsics(Stream::RIGHT, Stream::LEFT) << "}";

Reference result on Linux:

.. code-block:: bash

  $ ./samples/_output/bin/get_img_params
  I0510 15:00:22.643263  6980 utils.cc:26] Detecting MYNT EYE devices
  I0510 15:00:23.138811  6980 utils.cc:33] MYNT EYE devices:
  I0510 15:00:23.138849  6980 utils.cc:37]   index: 0, name: MYNT-EYE-S1000
  I0510 15:00:23.138855  6980 utils.cc:43] Only one MYNT EYE device, select index: 0
  I0510 15:00:23.210491  6980 get_img_params.cc:23] Intrinsics left: {width: 752, height: 480, fx: 736.38305001095545776, fy: 723.50066150722432212, cx: 356.91961817119693023, cy: 217.27271340923883258, model: 0, coeffs: [-0.54898645145016478, 0.52837141203888638, 0.00000000000000000, 0.00000000000000000, 0.00000000000000000]}
  I0510 15:00:23.210551  6980 get_img_params.cc:24] Intrinsics right: {width: 752, height: 480, fx: 736.38305001095545776, fy: 723.50066150722432212, cx: 456.68367112303980093, cy: 250.70083335536796199, model: 0, coeffs: [-0.51012886039889305, 0.38764476500996770, 0.00000000000000000, 0.00000000000000000, 0.00000000000000000]}
  I0510 15:00:23.210577  6980 get_img_params.cc:26] Extrinsics left to right: {rotation: [0.99701893306553813, -0.00095378124886237, -0.07715139279485062, 0.00144939967628305, 0.99997867219985104, 0.00636823256494144, 0.07714367342455503, -0.00646107164115277, 0.99699905125522237], translation: [-118.88991734400046596, -0.04560580387053091, -3.95313736911933855]}

Complete code examples, see `get_img_params.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/get_img_params.cc>`_ .
