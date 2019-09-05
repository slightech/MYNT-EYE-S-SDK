.. _data_get_depth:

Get Depth Image
===============

Depth images belongs to the upper layer of synthetic data. You need to start the ``EnableStreamData()`` beforehand, to get it through ``GetStreamData()``. The depth image type is CV_16UC1. In addition, it should be check not be empty before use.

For detailed process description, please see :ref:`get_stereo` :ref:`get_stereo_rectified`.

In addition, it is recommended to use plugins to calculate depth: Depth images work better and operate faster. Please refer to :ref:`get_with_plugin`.

Reference code snippet:

.. code-block:: c++

  auto &&api = API::Create(argc, argv);

  api->EnableStreamData(Stream::DEPTH);

  api->Start(Source::VIDEO_STREAMING);

  cv::namedWindow("frame");
  cv::namedWindow("depth");

  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);

    cv::Mat img;
    cv::hconcat(left_data.frame, right_data.frame, img);
    cv::imshow("frame", img);

    auto &&depth_data = api->GetStreamData(Stream::DEPTH);
    if (!depth_data.frame.empty()) {
      cv::imshow("depth", depth_data.frame);  // CV_16UC1
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  api->Stop(Source::VIDEO_STREAMING);

The above code uses OpenCV to display the image. When the display window is selected, pressing ``ESC/Q`` will end the program.

Complete code examples, see `get_depth.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/get_depth.cc>`_ .

.. tip::

Preview the value of a region of the depth image, see `get_depth_with_region.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/get_depth_with_region.cc>`_ .
