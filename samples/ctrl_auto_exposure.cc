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

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);
  if (!api) return 1;

  std::int32_t frame_rate = 0;
  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);

  Model model = api->GetModel();

  api->Start(Source::VIDEO_STREAMING);

  // Set auto exposure options fo s1030
  if (model == Model::STANDARD) {
    // auto-exposure: 0
    api->SetOptionValue(Option::EXPOSURE_MODE, 0);

    // max_gain: range [0,48], default 48
    api->SetOptionValue(Option::MAX_GAIN, 48);
    // max_exposure_time: range [0,240], default 240
    api->SetOptionValue(Option::MAX_EXPOSURE_TIME, 240);
    // desired_brightness: range [0,255], default 192
    api->SetOptionValue(Option::DESIRED_BRIGHTNESS, 192);

    frame_rate = api->GetOptionValue(Option::FRAME_RATE);

    LOG(INFO) << "Enable auto-exposure";
    LOG(INFO) << "Set EXPOSURE_MODE to "
              << api->GetOptionValue(Option::EXPOSURE_MODE);
    LOG(INFO) << "Set MAX_GAIN to " << api->GetOptionValue(Option::MAX_GAIN);
    LOG(INFO) << "Set MAX_EXPOSURE_TIME to "
              << api->GetOptionValue(Option::MAX_EXPOSURE_TIME);
    LOG(INFO) << "Set DESIRED_BRIGHTNESS to "
              << api->GetOptionValue(Option::DESIRED_BRIGHTNESS);
  }

  // Set auto exposure options fo S2000/S2100/S210A/S200B
  if (model == Model::STANDARD2 || model == Model::STANDARD210A ||
    model == Model::STANDARD200B) {
    // auto-exposure: 0
    api->SetOptionValue(Option::EXPOSURE_MODE, 0);

    // max_gain: range [0,255], default 8
    api->SetOptionValue(Option::MAX_GAIN, 8);
    // max_exposure_time: range [0,655], default 333
    api->SetOptionValue(Option::MAX_EXPOSURE_TIME, 333);
    // desired_brightness: range [1,255], default 122
    api->SetOptionValue(Option::DESIRED_BRIGHTNESS, 122);
    // min_exposure_time: range [0,655], default 0
    api->SetOptionValue(Option::MIN_EXPOSURE_TIME, 0);

    LOG(INFO) << "Enable auto-exposure";
    LOG(INFO) << "Set EXPOSURE_MODE to "
              << api->GetOptionValue(Option::EXPOSURE_MODE);
    LOG(INFO) << "Set MAX_GAIN to " << api->GetOptionValue(Option::MAX_GAIN);
    LOG(INFO) << "Set MAX_EXPOSURE_TIME to "
              << api->GetOptionValue(Option::MAX_EXPOSURE_TIME);
    LOG(INFO) << "Set DESIRED_BRIGHTNESS to "
              << api->GetOptionValue(Option::DESIRED_BRIGHTNESS);
    LOG(INFO) << "Set MIN_EXPOSURE_TIME to "
              << api->GetOptionValue(Option::MIN_EXPOSURE_TIME);
  }

  CVPainter painter(frame_rate);

  cv::namedWindow("frame");

  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);

    cv::Mat img;
    cv::hconcat(left_data.frame, right_data.frame, img);
    painter.DrawImgData(img, *left_data.img);
    cv::imshow("frame", img);

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  api->Stop(Source::VIDEO_STREAMING);
  return 0;
}
