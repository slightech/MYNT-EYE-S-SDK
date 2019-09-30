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
#include <memory>
#include <mutex>
#include <sstream>

#include <opencv2/highgui/highgui.hpp>

#include "mynteye/logger.h"
#include "mynteye/api/api.h"
#include "mynteye/util/times.h"

#include "util_cv.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);
  if (!api) return 1;

  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);

  Model model = api->GetModel();
  int imu_frequency = 200;
  // Set imu range for S1030
  if (model == Model::STANDARD) {
    // ACCELEROMETER_RANGE values: 4, 8, 16, 32
    api->SetOptionValue(Option::ACCELEROMETER_RANGE, 8);
    // GYROSCOPE_RANGE values: 500, 1000, 2000, 4000
    api->SetOptionValue(Option::GYROSCOPE_RANGE, 1000);
    imu_frequency = api->GetOptionValue(Option::IMU_FREQUENCY);
  }

  // Set imu range for S2000/S2100/S210A/S200B
  if (model == Model::STANDARD2 || model == Model::STANDARD210A ||
    model == Model::STANDARD200B) {
    // ACCELEROMETER_RANGE values: 6, 12, 24, 48
    api->SetOptionValue(Option::ACCELEROMETER_RANGE, 6);
    // GYROSCOPE_RANGE values: 250, 500, 1000, 2000, 4000
    api->SetOptionValue(Option::GYROSCOPE_RANGE, 1000);
  }

  LOG(INFO) << "Set ACCELEROMETER_RANGE to "
            << api->GetOptionValue(Option::ACCELEROMETER_RANGE);
  LOG(INFO) << "Set GYROSCOPE_RANGE to "
            << api->GetOptionValue(Option::GYROSCOPE_RANGE);

  // Count img
  std::atomic_uint img_count(0);
  api->SetStreamCallback(
      Stream::LEFT, [&img_count](const api::StreamData &data) {
        CHECK_NOTNULL(data.img);
        ++img_count;
      });

  // Count imu
  std::atomic_uint imu_count(0);
  std::shared_ptr<mynteye::ImuData> imu;
  std::mutex imu_mtx;
  CVPainter::angle_t gyro_offset;
  api->SetMotionCallback([&imu_count, &imu, &imu_mtx,
        &gyro_offset, &imu_frequency](const api::MotionData &data) {
    CHECK_NOTNULL(data.imu);
    ++imu_count;
    {
      std::lock_guard<std::mutex> _(imu_mtx);
      imu = data.imu;
      gyro_offset.angle_x += data.imu->gyro[0] / (1.0 * imu_frequency);
      gyro_offset.angle_y += data.imu->gyro[1] / (1.0 * imu_frequency);
      gyro_offset.angle_z += data.imu->gyro[2] / (1.0 * imu_frequency);
    }
  });

  api->Start(Source::ALL);

  CVPainter painter;

  cv::namedWindow("frame");

  auto &&time_beg = times::now();
  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);

    cv::Mat img;
    cv::hconcat(left_data.frame, right_data.frame, img);

    // Draw imu data
    if (imu) {
      std::lock_guard<std::mutex> _(imu_mtx);
      painter.DrawImuData(img, *imu, gyro_offset);
    }

    // Draw counts
    std::ostringstream ss;
    ss << "imu: " << imu_count;
    painter.DrawText(img, ss.str(), CVPainter::BOTTOM_RIGHT);

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
