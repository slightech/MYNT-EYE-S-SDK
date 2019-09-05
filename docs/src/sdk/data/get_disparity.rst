.. _data_get_disparity:

Get Disparity Image
===================

Disparity image belongs to the upper layer of synthetic data. You need to start the ``EnableStreamData()`` beforehand, to get it through ``GetStreamData()``. In addition, it should be check not be empty before use.

For detailed process description, please see :ref:`get_stereo` :ref:`get_stereo_rectified` .

It is recommended to use plugin to calculate depth: the depth map will be better with a higher frame rate. Please see :ref:`get_with_plugin` .

.. tip::
  The SetDisparityComputingMethodType method is used to change disparity computing method. Currently, BM and SGBM are available.

Reference code snippet:

.. code-block:: c++

  auto &&api = API::Create(argc, argv);

  // api->EnableStreamData(Stream::DISPARITY);
  api->EnableStreamData(Stream::DISPARITY_NORMALIZED);

  api->SetDisparityComputingMethodType(DisparityComputingMethod::BM);

  api->Start(Source::VIDEO_STREAMING);

  cv::namedWindow("frame");
  // cv::namedWindow("disparity");
  cv::namedWindow("disparity_normalized");

  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);

    cv::Mat img;
    cv::hconcat(left_data.frame, right_data.frame, img);
    cv::imshow("frame", img);

    // auto &&disp_data = api->GetStreamData(Stream::DISPARITY);
    // if (!disp_data.frame.empty()) {
    //   cv::imshow("disparity", disp_data.frame);
    // }

    auto &&disp_norm_data = api->GetStreamData(Stream::DISPARITY_NORMALIZED);
    if (!disp_norm_data.frame.empty()) {
      cv::imshow("disparity_normalized", disp_norm_data.frame);  // CV_8UC1
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  api->Stop(Source::VIDEO_STREAMING);

The above code uses OpenCV to display the image. Select the display window, press ``ESC/Q`` to exit in the program.

Complete code examples, see `get_disparity.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/get_disparity.cc>`_ .
