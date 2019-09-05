.. _data_save_single_image:

Save Single Image
=============================

Press "Space" "s" "S" to save image.

Reference commands:

.. code-block:: c++

  api->Start(Source::VIDEO_STREAMING);

  cv::namedWindow("frame");

  std::int32_t count = 0;
  std::cout << "Press 'Space' 's' 'S' to save image." << std::endl;
  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);
    if (!left_data.frame.empty() && !right_data.frame.empty()) {
      cv::Mat img;
      cv::hconcat(left_data.frame, right_data.frame, img);
      cv::imshow("frame", img);
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    } else if (key == 32 || key == 's' || key == 'S') {
      if (!left_data.frame.empty() && !right_data.frame.empty()) {
        char l_name[20];
        char r_name[20];
        ++count;
        snprintf(l_name, sizeof(l_name), "left_%d.jpg", count);
        snprintf(r_name, sizeof(r_name), "right_%d.jpg", count);

        cv::imwrite(l_name, left_data.frame);
        cv::imwrite(r_name, right_data.frame);

        std::cout << "Saved " << l_name << " " << r_name << " to current directory" << std::endl;
      }
    }
  }

  api->Stop(Source::VIDEO_STREAMING);

The above code uses OpenCV to display the image. When the display window is selected, pressing ``ESC/Q`` will end the program.

Complete code examples, see `save_single_image.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/save_single_image.cc>`_ .
