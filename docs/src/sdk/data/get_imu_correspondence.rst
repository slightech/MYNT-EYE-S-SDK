.. _data_get_imu_correspondence:

Get IMU Data With Timestamp Correspondence
===========================================

If wanna get image with timestamp in the middle of IMU datas, you could call `EnableTimestampCorrespondence()`` to enable this feature.

Reference code snippet:

.. code-block:: c++


  auto &&api = API::Create(argc, argv);

  // Enable motion datas with timestamp correspondence of some stream
  api->EnableTimestampCorrespondence(Stream::LEFT);

  api->Start(Source::ALL);

  cv::namedWindow("frame");

  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);

    auto img_stamp = left_data.img->timestamp;
    LOG(INFO) << "Img timestamp: " << img_stamp
        << ", diff_prev=" << (img_stamp - prev_img_stamp);
    prev_img_stamp = img_stamp;

    cv::Mat img;
    cv::hconcat(left_data.frame, right_data.frame, img);

    auto &&motion_datas = api->GetMotionDatas();
    LOG(INFO) << "Imu count: " << motion_datas.size();
    for (auto &&data : motion_datas) {
      auto imu_stamp = data.imu->timestamp;
      LOG(INFO) << "Imu timestamp: " << imu_stamp
          << ", diff_prev=" << (imu_stamp - prev_imu_stamp)
          << ", diff_img=" << (1.f + imu_stamp - img_stamp);
      prev_imu_stamp = imu_stamp;
    }
    LOG(INFO);

    cv::imshow("frame", img);

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  api->Stop(Source::ALL);

Reference result on Linux:

.. code-block:: bash

  $ ./samples/_output/bin/get_imu_correspondence
  I/utils.cc:30 Detecting MYNT EYE devices
  I/utils.cc:40 MYNT EYE devices:
  I/utils.cc:43   index: 0, name: MYNT-EYE-S1030, sn: 0281351000090807
  I/utils.cc:51 Only one MYNT EYE device, select index: 0
  I/synthetic.cc:126 camera calib model: kannala_brandt
  I/utils.cc:79 MYNT EYE devices:
  I/utils.cc:82   index: 0, request: width: 752, height: 480, format: Format::YUYV, fps: 60
  I/utils.cc:87 Only one stream request, select index: 0
  I/get_imu_correspondence.cc:50 Img timestamp: 171323050, diff_prev=39990
  I/get_imu_correspondence.cc:58 Imu count: 13
  I/get_imu_correspondence.cc:61 Imu timestamp: 171318710, diff_prev=171318710, diff_img=-4352
  I/get_imu_correspondence.cc:61 Imu timestamp: 171320730, diff_prev=2020, diff_img=-2320
  I/get_imu_correspondence.cc:61 Imu timestamp: 171322750, diff_prev=2020, diff_img=-304
  I/get_imu_correspondence.cc:61 Imu timestamp: 171324770, diff_prev=2020, diff_img=1712
  I/get_imu_correspondence.cc:61 Imu timestamp: 171326790, diff_prev=2020, diff_img=3728
  I/get_imu_correspondence.cc:61 Imu timestamp: 171328800, diff_prev=2010, diff_img=5744
  I/get_imu_correspondence.cc:61 Imu timestamp: 171330810, diff_prev=2010, diff_img=7760
  I/get_imu_correspondence.cc:61 Imu timestamp: 171332840, diff_prev=2030, diff_img=9776
  I/get_imu_correspondence.cc:61 Imu timestamp: 171334860, diff_prev=2020, diff_img=11808
  I/get_imu_correspondence.cc:61 Imu timestamp: 171336880, diff_prev=2020, diff_img=13824
  I/get_imu_correspondence.cc:61 Imu timestamp: 171338900, diff_prev=2020, diff_img=15840
  I/get_imu_correspondence.cc:61 Imu timestamp: 171340920, diff_prev=2020, diff_img=17872
  I/get_imu_correspondence.cc:61 Imu timestamp: 171342930, diff_prev=2010, diff_img=19872
  I/get_imu_correspondence.cc:66
  I/get_imu_correspondence.cc:50 Img timestamp: 171403040, diff_prev=79990
  I/get_imu_correspondence.cc:58 Imu count: 20
  I/get_imu_correspondence.cc:61 Imu timestamp: 171383310, diff_prev=40380, diff_img=-19728
  I/get_imu_correspondence.cc:61 Imu timestamp: 171385330, diff_prev=2020, diff_img=-17712
  I/get_imu_correspondence.cc:61 Imu timestamp: 171387350, diff_prev=2020, diff_img=-15696
  I/get_imu_correspondence.cc:61 Imu timestamp: 171389370, diff_prev=2020, diff_img=-13664
  I/get_imu_correspondence.cc:61 Imu timestamp: 171391380, diff_prev=2010, diff_img=-11664
  I/get_imu_correspondence.cc:61 Imu timestamp: 171393390, diff_prev=2010, diff_img=-9648
  I/get_imu_correspondence.cc:61 Imu timestamp: 171395420, diff_prev=2030, diff_img=-7616
  I/get_imu_correspondence.cc:61 Imu timestamp: 171397440, diff_prev=2020, diff_img=-5600
  I/get_imu_correspondence.cc:61 Imu timestamp: 171399460, diff_prev=2020, diff_img=-3584
  I/get_imu_correspondence.cc:61 Imu timestamp: 171401480, diff_prev=2020, diff_img=-1568
  I/get_imu_correspondence.cc:61 Imu timestamp: 171403500, diff_prev=2020, diff_img=464
  I/get_imu_correspondence.cc:61 Imu timestamp: 171405510, diff_prev=2010, diff_img=2464
  I/get_imu_correspondence.cc:61 Imu timestamp: 171407520, diff_prev=2010, diff_img=4480
  I/get_imu_correspondence.cc:61 Imu timestamp: 171409540, diff_prev=2020, diff_img=6496
  I/get_imu_correspondence.cc:61 Imu timestamp: 171411570, diff_prev=2030, diff_img=8528
  I/get_imu_correspondence.cc:61 Imu timestamp: 171413590, diff_prev=2020, diff_img=10544
  I/get_imu_correspondence.cc:61 Imu timestamp: 171415610, diff_prev=2020, diff_img=12576
  I/get_imu_correspondence.cc:61 Imu timestamp: 171417630, diff_prev=2020, diff_img=14592
  I/get_imu_correspondence.cc:61 Imu timestamp: 171419650, diff_prev=2020, diff_img=16608
  I/get_imu_correspondence.cc:61 Imu timestamp: 171421660, diff_prev=2010, diff_img=18624

Complete code examples, see `get_imu_correspondence.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/get_imu_correspondence.cc>`_ .
