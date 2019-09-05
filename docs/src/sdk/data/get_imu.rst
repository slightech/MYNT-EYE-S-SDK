.. _data_get_imu_data:

Get IMU Data
=============

The API offers ``Start()`` / ``Stop()`` function to start/stop capturing data. You can set the argument to``Source::MOTION_TRACKING`` to capture IMU data only, or set it to ``Source::ALL`` to capture both image and IMU data.

During capturing data, you need ``EnableMotionDatas()`` to enable caching in order to get IMU data from ``GetMotionDatas()`` . Otherwise, IMU data is only available through the callback interface, see :ref:`get_from_callbacks` .

Sample code snippet:

.. code-block:: c++

  auto &&api = API::Create(argc, argv);

  // Enable this will cache the motion datas until you get them.
  api->EnableMotionDatas();

  api->Start(Source::ALL);

  CVPainter painter;

  cv::namedWindow("frame");

  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);

    cv::Mat img;
    cv::hconcat(left_data.frame, right_data.frame, img);

    auto &&motion_datas = api->GetMotionDatas();
    /*
    for (auto &&data : motion_datas) {
      LOG(INFO) << "Imu frame_id: " << data.imu->frame_id
                << ", timestamp: " << data.imu->timestamp
                << ", accel_x: " << data.imu->accel[0]
                << ", accel_y: " << data.imu->accel[1]
                << ", accel_z: " << data.imu->accel[2]
                << ", gyro_x: " << data.imu->gyro[0]
                << ", gyro_y: " << data.imu->gyro[1]
                << ", gyro_z: " << data.imu->gyro[2]
                << ", temperature: " << data.imu->temperature;
    }
    */

    painter.DrawImgData(img, *left_data.img);
    static std::vector<api::MotionData> motion_datas_s = motion_datas;

    if (!motion_datas.empty() && motion_datas.size() > 0) {
      motion_datas_s = motion_datas;
    }
    if (!motion_datas_s.empty() && motion_datas_s.size() > 0) {
      painter.DrawImuData(img, *motion_datas_s[0].imu);
    }

    cv::imshow("frame", img);

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  api->Stop(Source::ALL);

OpenCV is used to display image and data. When window is selected, press ``ESC/Q`` to exit program.

Complete code examples, see `get_imu.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/get_imu.cc>`_ .
