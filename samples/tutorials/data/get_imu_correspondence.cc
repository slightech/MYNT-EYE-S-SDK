// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <opencv2/highgui/highgui.hpp>

#include "mynteye/logger.h"
#include "mynteye/api/api.h"

#include "util/cv_painter.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);
  if (!api) return 1;

  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);

  // Enable motion datas with timestamp correspondence of some stream
  api->EnableStreamData(Stream::LEFT);
  api->EnableStreamData(Stream::RIGHT);
  api->EnableTimestampCorrespondence(Stream::LEFT);

  api->Start(Source::ALL);

  CVPainter painter;

  cv::namedWindow("frame");

  std::uint64_t prev_img_stamp = 0;
  std::uint64_t prev_imu_stamp = 0;
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

    /*
    painter.DrawImgData(img, *left_data.img);
    if (!motion_datas.empty()) {
      painter.DrawImuData(img, *motion_datas[0].imu);
    }
    */

    cv::imshow("frame", img);

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  api->Stop(Source::ALL);
  return 0;
}
