.. _data_get_stereo:

Get Original Binocular Image
=============================

Use ``Start()`` or ``Stop()`` , to start or stop data capturing. If you only need the image data, use ``Source::VIDEO_STREAMING`` .

When data capturing starts, call ``WaitForStreams()`` function. Once data capturing begins, use ``GetStreamData()`` to get your data.

Reference commands:

.. code-block:: c++

  auto &&api = API::Create(argc, argv);

  api->Start(Source::VIDEO_STREAMING);

  cv::namedWindow("frame");

  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);

    cv::Mat img;
    cv::hconcat(left_data.frame, right_data.frame, img);
    cv::imshow("frame", img);

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  api->Stop(Source::VIDEO_STREAMING);

The above code uses OpenCV to display the image. When the display window is selected, pressing ``ESC/Q`` will end the program.

Complete code examples, see `get_stereo.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/camera_with_junior_device_api.cc>`_ .
