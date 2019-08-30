.. _data_get_from_callbacks:

Get Data From Callbacks
========================

API offers function ``SetStreamCallback()`` and ``SetMotionCallback()`` to set callbacks for various data.

.. attention::

  Make sure to not block callback. If the data processing time is too long, use the callback as a data producer.

Reference code snippet:

.. code-block:: c++

  auto &&api = API::Create(argc, argv);

  // Attention: must not block the callbacks.

  // Get left image from callback
  std::atomic_uint left_count(0);
  api->SetStreamCallback(
      Stream::LEFT, [&left_count](const api::StreamData &data) {
        CHECK_NOTNULL(data.img);
        ++left_count;
      });

  // Get depth image from callback
  api->EnableStreamData(Stream::DEPTH);
  std::atomic_uint depth_count(0);
  cv::Mat depth;
  std::mutex depth_mtx;
  api->SetStreamCallback(
      Stream::DEPTH,
      [&depth_count, &depth, &depth_mtx](const api::StreamData &data) {
        UNUSED(data)
        ++depth_count;
        {
          std::lock_guard<std::mutex> _(depth_mtx);
          depth = data.frame;
        }
      });

  // Get motion data from callback
  std::atomic_uint imu_count(0);
  std::shared_ptr<mynteye::ImuData> imu;
  std::mutex imu_mtx;
  api->SetMotionCallback(
      [&imu_count, &imu, &imu_mtx](const api::MotionData &data) {
        CHECK_NOTNULL(data.imu);
        ++imu_count;
        {
          std::lock_guard<std::mutex> _(imu_mtx);
          imu = data.imu;
        }
      });

  api->Start(Source::ALL);

  CVPainter painter;

  cv::namedWindow("frame");
  cv::namedWindow("depth");

  unsigned int depth_num = 0;
  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);

    // Concat left and right as img
    cv::Mat img;
    cv::hconcat(left_data.frame, right_data.frame, img);

    // Draw img data and size
    painter.DrawImgData(img, *left_data.img);

    // Draw imu data
    if (imu) {
      std::lock_guard<std::mutex> _(imu_mtx);
      painter.DrawImuData(img, *imu);
    }

    // Draw counts
    std::ostringstream ss;
    ss << "left: " << left_count << ", depth: " << depth_count
       << ", imu: " << imu_count;
    painter.DrawText(img, ss.str(), CVPainter::BOTTOM_RIGHT);

    // Show img
    cv::imshow("frame", img);

    // Show depth
    if (!depth.empty()) {
      // Is the depth a new one?
      if (depth_num != depth_count || depth_num == 0) {
        std::lock_guard<std::mutex> _(depth_mtx);
        depth_num = depth_count;
        // LOG(INFO) << "depth_num: " << depth_num;
        ss.str("");
        ss.clear();
        ss << "depth: " << depth_count;
        painter.DrawText(depth, ss.str());
        cv::imshow("depth", depth);  // CV_16UC1
      }
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  api->Stop(Source::ALL);

OpenCV is used to display images and data above. When the window is selected, pressing ``ESC/Q`` will exit program.

Complete code examples, see `get_from_callbacks.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/get_from_callbacks.cc>`_ .
