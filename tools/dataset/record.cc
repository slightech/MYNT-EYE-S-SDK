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
#include "mynteye/api/api.h"
#include "mynteye/util/times.h"

#include "dataset/dataset.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);
  if (!api)
    return 1;
  api->InitResolution(Resolution::RES_1280x400);
  api->SetStreamRequest(Format::BGR888, FrameRate::RATE_30_FPS);
  api->LogOptionInfos();

  // Enable this will cache the motion datas until you get them.
  api->EnableMotionDatas();
  api->Start(Source::ALL);

  const char *outdir;
  if (argc >= 2) {
    outdir = argv[1];
  } else {
    outdir = "./dataset";
  }
  tools::Dataset dataset(outdir);

  cv::namedWindow("frame");

  std::size_t img_count = 0;
  std::size_t imu_count = 0;
  auto &&time_beg = times::now();
  while (true) {
    api->WaitForStreams();

    auto &&left_datas = api->GetStreamDatas(Stream::LEFT);
    auto &&right_datas = api->GetStreamDatas(Stream::RIGHT);
    img_count += left_datas.size();

    auto &&motion_datas = api->GetMotionDatas();
    imu_count += motion_datas.size();

    auto &&left_frame = left_datas.back().frame;
    auto &&right_frame = right_datas.back().frame;

    cv::Mat img;

    if (left_frame->format() == Format::GREY) {
      cv::Mat left_img(
          left_frame->height(), left_frame->width(), CV_8UC1,
          left_frame->data());
      cv::Mat right_img(
          right_frame->height(), right_frame->width(), CV_8UC1,
          right_frame->data());
      cv::hconcat(left_img, right_img, img);
    } else if (left_frame->format() == Format::YUYV) {
      cv::Mat left_img(
          left_frame->height(), left_frame->width(), CV_8UC2,
          left_frame->data());
      cv::Mat right_img(
          right_frame->height(), right_frame->width(), CV_8UC2,
          right_frame->data());
      cv::cvtColor(left_img, left_img, cv::COLOR_YUV2BGR_YUY2);
      cv::cvtColor(right_img, right_img, cv::COLOR_YUV2BGR_YUY2);
      cv::hconcat(left_img, right_img, img);
    } else if (left_frame->format() == Format::BGR888) {
      cv::Mat left_img(
          left_frame->height(), left_frame->width(), CV_8UC3,
          left_frame->data());
      cv::Mat right_img(
          right_frame->height(), right_frame->width(), CV_8UC3,
          right_frame->data());
      cv::hconcat(left_img, right_img, img);
    } else {
      return -1;
    }
    cv::imshow("frame", img);
    if (img_count > 10 && imu_count > 50) {  // save
      for (auto &&left : left_datas) {
        dataset.SaveStreamData(Stream::LEFT, left);
      }
      for (auto &&right : right_datas) {
        dataset.SaveStreamData(Stream::RIGHT, right);
      }

      for (auto &&motion : motion_datas) {
        dataset.SaveMotionData(motion);
      }

      std::cout << "\rSaved " << img_count << " imgs"
                << ", " << imu_count << " imus" << std::flush;
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }
  std::cout << " to " << outdir << std::endl;
  auto &&time_end = times::now();

  api->Stop(Source::ALL);

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
