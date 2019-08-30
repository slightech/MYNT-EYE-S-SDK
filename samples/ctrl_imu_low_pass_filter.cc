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
#include <atomic>

#include <opencv2/highgui/highgui.hpp>

#include "mynteye/logger.h"
#include "mynteye/api/api.h"
#include "mynteye/util/times.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);
  if (!api) return 1;

  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);

  Model model = api->GetModel();

  // MYNTEYE-S1030 don't support this option
  if (model == Model::STANDARD) {
    LOG(INFO) << "Sorry,MYNTEYE-S1030 don't support imu low pass settings";
    return 0;
  }

  // Set imu low pass filter for S2000/S2100/S210A/S200B
  if (model == Model::STANDARD2 ||
   model == Model::STANDARD210A || model == Model::STANDARD200B) {
    // ACCELEROMETER_RANGE values: 0, 1, 2
    api->SetOptionValue(Option::ACCELEROMETER_LOW_PASS_FILTER, 2);
    // GYROSCOPE_RANGE values: 23, 64
    api->SetOptionValue(Option::GYROSCOPE_LOW_PASS_FILTER, 64);

    LOG(INFO) << "Set ACCELEROMETER_LOW_PASS_FILTER to "
              << api->GetOptionValue(Option::ACCELEROMETER_LOW_PASS_FILTER);
    LOG(INFO) << "Set GYROSCOPE_LOW_PASS_FILTER to "
              << api->GetOptionValue(Option::GYROSCOPE_LOW_PASS_FILTER);
  }

  // Count img
  std::atomic_uint img_count(0);
  api->SetStreamCallback(
      Stream::LEFT, [&img_count](const api::StreamData &data) {
        CHECK_NOTNULL(data.img);
        ++img_count;
      });

  // Count imu
  std::atomic_uint imu_count(0);
  api->SetMotionCallback([&imu_count](const api::MotionData &data) {
    CHECK_NOTNULL(data.imu);
    ++imu_count;
  });

  api->Start(Source::ALL);

  cv::namedWindow("frame");

  auto &&time_beg = times::now();
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
  auto &&time_end = times::now();

  api->Stop(Source::ALL);

  // Calculate img fps and imu hz
  float elapsed_ms =
      times::count<times::microseconds>(time_end - time_beg) * 0.001f;
  LOG(INFO) << "Time beg: " << times::to_local_string(time_beg)
            << ", end: " << times::to_local_string(time_end)
            << ", cost: " << elapsed_ms << "ms";
  LOG(INFO) << "Img count: " << img_count
            << ", fps: " << (1000.f * img_count / elapsed_ms);
  LOG(INFO) << "Imu count: " << imu_count
            << ", hz: " << (1000.f * imu_count / elapsed_ms);
  return 0;
}
