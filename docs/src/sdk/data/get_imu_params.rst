.. _data_get_imu_params:

Get IMU Calibration Parameters
==============================

Use ``GetMotionIntrinsics()`` & ``GetMotionExtrinsics()`` to get current IMU calibration parameters.

Reference commands:

.. code-block:: c++

  auto &&api = API::Create(argc, argv);

  LOG(INFO) << "Motion intrinsics: {" << api->GetMotionIntrinsics() << "}";
  LOG(INFO) << "Motion extrinsics left to imu: {"
            << api->GetMotionExtrinsics(Stream::LEFT) << "}";

Complete code examples, see `get_imu_params.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/get_imu_params.cc>`_ .
