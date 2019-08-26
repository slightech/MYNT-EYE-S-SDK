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
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>

#include "mynteye/api/api.h"
#include "mynteye/logger.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);
  if (!api) return 1;

  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);

  api->SetDisparityComputingMethodType(DisparityComputingMethod::SGBM);
  api->EnableStreamData(Stream::DISPARITY_NORMALIZED);

  if (argc == 2) {
    std::string config_path(argv[1]);
    if (api->ConfigDisparityFromFile(config_path)) {
      LOG(INFO) << "load disparity file: "
                << config_path
                << " success."
                << std::endl;
    } else {
      LOG(INFO) << "load disparity file: "
                << config_path
                << " failed."
                << std::endl;
    }
  }

  api->Start(Source::VIDEO_STREAMING);

  cv::namedWindow("frame");
  // cv::namedWindow("disparity");
  cv::namedWindow("disparity_normalized");

  double fps;
  double t = 0.01;
  std::cout << "disparity fps:" << std::endl;

  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);

    if (!left_data.frame.empty() && !right_data.frame.empty()) {
      cv::Mat img;
      cv::hconcat(left_data.frame, right_data.frame, img);
      cv::imshow("frame", img);
    }

    // auto &&disp_data = api->GetStreamData(Stream::DISPARITY);
    // if (!disp_data.frame.empty()) {
    //   cv::imshow("disparity", disp_data.frame);
    // }

    auto &&disp_norm_data = api->GetStreamData(Stream::DISPARITY_NORMALIZED);
    if (!disp_norm_data.frame.empty()) {
      double t_c = cv::getTickCount() / cv::getTickFrequency();
      fps = 1.0/(t_c - t);
      printf("\b\b\b\b\b\b\b\b\b%.2f", fps);
      t = t_c;
      cv::imshow("disparity_normalized", disp_norm_data.frame);  // CV_8UC1
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  api->Stop(Source::VIDEO_STREAMING);
  return 0;
}
