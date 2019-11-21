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
#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye/logger.h"
#include "mynteye/device/device.h"
#include "mynteye/device/utils.h"
#include "mynteye/util/times.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  glog_init _(argc, argv);

  auto &&device = device::select();
  if (!device) return 1;

  bool ok;
  auto &&request = device::select_request(device, &ok);
  if (!ok) return 1;
  device->ConfigStreamRequest(request);

  std::size_t left_count = 0;
  device->SetStreamCallback(
      Stream::LEFT, [&left_count](const device::StreamData &data) {
        CHECK_NOTNULL(data.img);
        ++left_count;
        VLOG(2) << Stream::LEFT << "count: " << left_count;
        VLOG(2) << "  frame_id: " << data.img->frame_id
                << ", timestamp: " << data.img->timestamp
                << ", exposure_time: " << data.img->exposure_time;
      });
  std::size_t right_count = 0;
  device->SetStreamCallback(
      Stream::RIGHT, [&right_count](const device::StreamData &data) {
        CHECK_NOTNULL(data.img);
        ++right_count;
        VLOG(2) << Stream::RIGHT << "count: " << right_count;
        VLOG(2) << "  frame_id: " << data.img->frame_id
                << ", timestamp: " << data.img->timestamp
                << ", exposure_time: " << data.img->exposure_time;
      });

  std::size_t imu_count = 0;

  device->SetMotionCallback([&imu_count](const device::MotionData &data) {
    CHECK_NOTNULL(data.imu);
    ++imu_count;
    VLOG(2) << "Imu count: " << imu_count;
    VLOG(2) << ", timestamp: " << data.imu->timestamp
            << ", accel_x: " << data.imu->accel[0]
            << ", accel_y: " << data.imu->accel[1]
            << ", accel_z: " << data.imu->accel[2]
            << ", gyro_x: " << data.imu->gyro[0]
            << ", gyro_y: " << data.imu->gyro[1]
            << ", gyro_z: " << data.imu->gyro[2]
            << ", temperature: " << data.imu->temperature;
  });

  // Enable this will cache the motion datas until you get them.
  device->EnableMotionDatas();
  device->Start(Source::ALL);
  cv::namedWindow("frame");

  std::size_t motion_count = 0;
  auto &&time_beg = times::now();
  while (true) {
    device->WaitForStreams();

    device::StreamData left_data = device->GetStreamData(Stream::LEFT);
    device::StreamData right_data = device->GetStreamData(Stream::RIGHT);
    auto left_data_frame = *(left_data.frame);
    auto right_data_frame = *(right_data.frame);

    auto &&motion_datas = device->GetMotionDatas();
    motion_count += motion_datas.size();
    for (auto &&data : motion_datas) {
      LOG(INFO) << "timestamp: " << data.imu->timestamp
                << ", accel_x: " << data.imu->accel[0]
                << ", accel_y: " << data.imu->accel[1]
                << ", accel_z: " << data.imu->accel[2]
                << ", gyro_x: " << data.imu->gyro[0]
                << ", gyro_y: " << data.imu->gyro[1]
                << ", gyro_z: " << data.imu->gyro[2]
                << ", temperature: " << data.imu->temperature;
    }

    cv::Mat img;

    // TODO(Kalman): Extract into public or internal method
    if (left_data_frame.format() == Format::GREY) {
      cv::Mat left_img(
          left_data_frame.height(), left_data_frame.width(), CV_8UC1,
          left_data_frame.data());
      cv::Mat right_img(
          right_data_frame.height(), right_data_frame.width(), CV_8UC1,
          right_data_frame.data());
      cv::hconcat(left_img, right_img, img);
    } else if (left_data_frame.format() == Format::YUYV) {
      cv::Mat left_img(
          left_data_frame.height(), left_data_frame.width(), CV_8UC2,
          left_data_frame.data());
      cv::Mat right_img(
          right_data_frame.height(), right_data_frame.width(), CV_8UC2,
          right_data_frame.data());
      cv::cvtColor(left_img, left_img, cv::COLOR_YUV2BGR_YUY2);
      cv::cvtColor(right_img, right_img, cv::COLOR_YUV2BGR_YUY2);
      cv::hconcat(left_img, right_img, img);
    } else if (left_data_frame.format() == Format::BGR888) {
      cv::Mat left_img(
          left_data_frame.height(), left_data_frame.width(), CV_8UC3,
          left_data_frame.data());
      cv::Mat right_img(
          right_data_frame.height(), right_data_frame.width(), CV_8UC3,
          right_data_frame.data());
      cv::hconcat(left_img, right_img, img);
    } else {
      return -1;
    }

    cv::imshow("frame", img);

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }
  auto &&time_end = times::now();

  device->Stop(Source::ALL);

  float elapsed_ms =
      times::count<times::microseconds>(time_end - time_beg) * 0.001f;
  LOG(INFO) << "Time beg: " << times::to_local_string(time_beg)
            << ", end: " << times::to_local_string(time_end)
            << ", cost: " << elapsed_ms << "ms";
  LOG(INFO) << "Left count: " << left_count
            << ", fps: " << (1000.f * left_count / elapsed_ms);
  LOG(INFO) << "Right count: " << right_count
            << ", fps: " << (1000.f * right_count / elapsed_ms);
  LOG(INFO) << "Imu count: " << imu_count
            << ", hz: " << (1000.f * imu_count / elapsed_ms);
  LOG(INFO) << "Motion count: " << motion_count
            << ", hz: " << (1000.f * motion_count / elapsed_ms);
  return 0;
}
