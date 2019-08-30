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

#include "util_cv.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);
  if (!api) return 1;

  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);

  // Attention: must not block the callbacks.

  // Get left image from callback
  std::atomic_uint left_count(0);
  api->SetStreamCallback(
      Stream::LEFT, [&left_count](const api::StreamData &data) {
        CHECK_NOTNULL(data.img);
        ++left_count;
        // LOG(INFO) << Stream::LEFT << ", count: " << left_count;
        // LOG(INFO) << "  frame_id: " << data.img->frame_id
        //           << ", timestamp: " << data.img->timestamp
        //           << ", exposure_time: " << data.img->exposure_time;
      });

  // Get depth image from callback
  api->EnableStreamData(Stream::DEPTH);
  std::atomic_uint depth_count(0);
  cv::Mat depth;
  std::mutex depth_mtx;
  api->SetStreamCallback(
      Stream::DEPTH,
      [&depth_count, &depth, &depth_mtx](const api::StreamData &data) {
        MYNTEYE_UNUSED(data)
        ++depth_count;
        {
          std::lock_guard<std::mutex> _(depth_mtx);
          depth = data.frame;
        }
        // LOG(INFO) << Stream::DEPTH << ", count: " << depth_count;
      });

  // Get motion data from callback
  std::atomic_uint imu_count(0);
  std::shared_ptr<mynteye::ImuData> imu;
  std::mutex imu_mtx;
  api->SetMotionCallback(
      [&imu_count, &imu, &imu_mtx](const api::MotionData &data) {
        CHECK_NOTNULL(data.imu);
        ++imu_count;
        {
          std::lock_guard<std::mutex> _(imu_mtx);
          imu = data.imu;
        }
        // LOG(INFO) << "Imu count: " << imu_count;
        // LOG(INFO) << "  frame_id: " << data.imu->frame_id
        //           << ", timestamp: " << data.imu->timestamp
        //           << ", accel_x: " << data.imu->accel[0]
        //           << ", accel_y: " << data.imu->accel[1]
        //           << ", accel_z: " << data.imu->accel[2]
        //           << ", gyro_x: " << data.imu->gyro[0]
        //           << ", gyro_y: " << data.imu->gyro[1]
        //           << ", gyro_z: " << data.imu->gyro[2]
        //           << ", temperature: " << data.imu->temperature;
      });

  api->Start(Source::ALL);

  CVPainter painter;

  cv::namedWindow("frame");
  cv::namedWindow("depth");

  unsigned int depth_num = 0;
  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);

    // Concat left and right as img
    cv::Mat img;
    cv::hconcat(left_data.frame, right_data.frame, img);

    // Draw img data and size
    painter.DrawImgData(img, *left_data.img);

    // Draw imu data
    if (imu) {
      std::lock_guard<std::mutex> _(imu_mtx);
      painter.DrawImuData(img, *imu);
    }

    // Draw counts
    std::ostringstream ss;
    ss << "left: " << left_count << ", depth: " << depth_count
       << ", imu: " << imu_count;
    painter.DrawText(img, ss.str(), CVPainter::BOTTOM_RIGHT);

    // Show img
    cv::imshow("frame", img);

    // Show depth
    if (!depth.empty()) {
      // Is the depth a new one?
      if (depth_num != depth_count || depth_num == 0) {
        std::lock_guard<std::mutex> _(depth_mtx);
        depth_num = depth_count;
        // LOG(INFO) << "depth_num: " << depth_num;
        ss.str("");
        ss.clear();
        ss << "depth: " << depth_count;
        painter.DrawText(depth, ss.str());
        cv::imshow("depth", depth);  // CV_16UC1
      }
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  api->Stop(Source::ALL);
  return 0;
}
