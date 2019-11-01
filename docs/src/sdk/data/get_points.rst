.. _data_get_points:

Get Point Image
================

Point images belongs to upper layer of synthetic data. To get this kind of data through ``GetStreamData()``, you need to start the ``EnableStreamData()`` beforehand. It should be check not empty before use.

For detail process description, please see :ref:`get_stereo` :ref:`get_stereo_rectified` .


Sample code snippet:

.. code-block:: c++

  auto &&api = API::Create(argc, argv);

  api->EnableStreamData(Stream::POINTS);

  api->Start(Source::VIDEO_STREAMING);

  cv::namedWindow("frame");
  PCViewer pcviewer;

  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);

    cv::Mat img;
    cv::hconcat(left_data.frame, right_data.frame, img);
    cv::imshow("frame", img);

    auto &&points_data = api->GetStreamData(Stream::POINTS);
    if (!points_data.frame.empty()) {
      pcviewer.Update(points_data.frame);
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
    if (pcviewer.WasStopped()) {
      break;
    }
  }

  api->Stop(Source::VIDEO_STREAMING);

`PCL <https://github.com/PointCloudLibrary/pcl>`_ is used to display point images above. Program will close when point image window is closed.

Complete code examples, see `get_depth_and_points.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/get_depth_and_points.cc>`_ .

.. attention::

  Sample code only compiles when `PCL <https://github.com/PointCloudLibrary/pcl>`_ is ready. If your PCL was installed in a different directory, please set ``CMAKE_PREFIX_PATH`` in `CMakeLists.txt <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/CMakeLists.txt>`_ to the path of ``PCLConfig.cmake`` . You can find ``CMAKE_PREFIX_PATH`` near ``find_package(PCL)`` .
