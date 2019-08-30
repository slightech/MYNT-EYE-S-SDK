.. _data_get_stereo_rectified:

Get Stereo Camera Correction Image
==================================

The ``GetStreamData()`` API provided can only get the raw data of the hardware, for example, the stereo camera raw image.

The stereo camera correction image belongs to the upper layer of synthetic data. For such data, you need to enable ``EnableStreamData()`` before you can get ``GetStreamData()``.

In addition, ``WaitForStreams()`` waits for the key of the raw data. At the beginning when the synthetic data may still being processed, the value taken out will be null, so it needs to check not empty.

.. tip::

  If you want the synthetic data once it is generated, see :ref:`get_from_callbacks`.

Reference code snippet:

.. code-block:: c++

  auto &&api = API::Create(argc, argv);

  api->EnableStreamData(Stream::LEFT_RECTIFIED);
  api->EnableStreamData(Stream::RIGHT_RECTIFIED);

  api->Start(Source::VIDEO_STREAMING);

  cv::namedWindow("frame");

  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT_RECTIFIED);
    auto &&right_data = api->GetStreamData(Stream::RIGHT_RECTIFIED);

    if (!left_data.frame.empty() && !right_data.frame.empty()) {
      cv::Mat img;
      cv::hconcat(left_data.frame, right_data.frame, img);
      cv::imshow("frame", img);
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  api->Stop(Source::VIDEO_STREAMING);

OpenCV is used to display the image above. Select the display window, press ``ESC/Q`` to exit the program.

Complete code examples, see `get_stereo_rectified.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/get_stereo_rectified.cc>`_ .
