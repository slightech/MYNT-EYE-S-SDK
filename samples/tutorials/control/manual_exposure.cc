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

<<<<<<< HEAD
#include "mynteye/api.h"
#include "mynteye/logger.h"
=======
#include "mynteye/logger.h"
#include "mynteye/api/api.h"
>>>>>>> origin/develop

#include "util/cv_painter.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);
  if (!api)
    return 1;

  // manual-exposure: 1
  api->SetOptionValue(Option::EXPOSURE_MODE, 1);

  // gain: range [0,48], default 24
  api->SetOptionValue(Option::GAIN, 24);
  // brightness/exposure_time: range [0,240], default 120
  api->SetOptionValue(Option::BRIGHTNESS, 120);
  // contrast/black_level_calibration: range [0,255], default 127
  api->SetOptionValue(Option::CONTRAST, 127);

  LOG(INFO) << "Enable manual-exposure";
  LOG(INFO) << "Set GAIN to " << api->GetOptionValue(Option::GAIN);
  LOG(INFO) << "Set BRIGHTNESS to " << api->GetOptionValue(Option::BRIGHTNESS);
  LOG(INFO) << "Set CONTRAST to " << api->GetOptionValue(Option::CONTRAST);

  api->Start(Source::VIDEO_STREAMING);

  CVPainter painter(api->GetOptionValue(Option::FRAME_RATE));

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
