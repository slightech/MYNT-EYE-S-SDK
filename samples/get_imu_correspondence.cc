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

#include "util_cv.h"

// #define CHECK_ACCEL_THEN_GYRO
#define SEQ_FIRST  1  // accel
#define SEQ_SECOND 2  // gyro

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);
  if (!api) return 1;

  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);

  // Enable motion datas with timestamp correspondence of some stream
  api->EnableTimestampCorrespondence(Stream::LEFT);

  api->Start(Source::ALL);

  CVPainter painter;

  cv::namedWindow("frame");

  std::uint64_t prev_img_stamp = 0;
  std::uint64_t prev_imu_stamp = 0;
#ifdef CHECK_ACCEL_THEN_GYRO
  std::uint8_t prev_imu_flag = 0;
  std::uint64_t imu_count = 0;
  std::uint64_t imu_disorder_count = 0;
  bool exit = false;
#endif
  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);
    if (!left_data.frame.empty() || !right_data.frame.empty()) {
      auto img_stamp = left_data.img->timestamp;
      LOG(INFO) << "Img timestamp: " << img_stamp
          << ", diff_prev=" << (img_stamp - prev_img_stamp);
      prev_img_stamp = img_stamp;

      cv::Mat img;
      cv::hconcat(left_data.frame, right_data.frame, img);

      auto &&motion_datas = api->GetMotionDatas();
      LOG(INFO) << "Imu count: " << motion_datas.size();
      if (motion_datas.size() == 0) {
        continue;
      }
      for (size_t i = 0, n = motion_datas.size() - 1; i <= n; ++i) {
        auto data = motion_datas[i];
        auto imu_flag = data.imu->flag;
        auto imu_stamp = data.imu->timestamp;

        std::stringstream ss;
        if (imu_flag == 0) {  // accel + gyro
          ss << "Imu";
        } else if (imu_flag == 1) {  // accel
          ss << "Accel";
        } else if (imu_flag == 2) {  // gyro
          ss << "Gyro";
        }
        ss << " timestamp: " << imu_stamp
            << ", diff_prev=" << (imu_stamp - prev_imu_stamp)
            << ", diff_img=" << (1.0f + imu_stamp - img_stamp);
  #ifdef CHECK_ACCEL_THEN_GYRO
        if (imu_flag > 0) {
          bool ok = false;
          if (i == 0) {  // first
            ok = (imu_flag == SEQ_FIRST);
          } else if (i == n) {  // last
            ok = (imu_flag == SEQ_SECOND);
          } else {
            if (imu_flag == SEQ_FIRST) {
              ok = (prev_imu_flag == SEQ_SECOND);
            } else if (imu_flag == SEQ_SECOND) {
              ok = (prev_imu_flag == SEQ_FIRST);
            }
          }
          ss << (ok ? " ✓" : " x");
          if (!ok) ++imu_disorder_count;
          prev_imu_flag = imu_flag;
          if (!exit) {
            if (!ok) exit = true;
          }
        }
  #endif
        LOG(INFO) << ss.str();

        prev_imu_stamp = imu_stamp;
      }
      LOG(INFO);
  #ifdef CHECK_ACCEL_THEN_GYRO
      imu_count += motion_datas.size();
      if (exit) break;
  #endif

      /*
      painter.DrawImgData(img, *left_data.img);
      if (!motion_datas.empty() && motion_datas.size() > 0) {
        painter.DrawImuData(img, *motion_datas[0].imu);
      }
      */

      cv::imshow("frame", img);
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  api->Stop(Source::ALL);

#ifdef CHECK_ACCEL_THEN_GYRO
  if (imu_disorder_count > 0) {
    LOG(INFO) << "accel_then_gyro, disorder_count: " << imu_disorder_count
        << "/" << imu_count;
  } else {
    LOG(INFO) << "accel_then_gyro, ok";
  }
#endif
  return 0;
}
