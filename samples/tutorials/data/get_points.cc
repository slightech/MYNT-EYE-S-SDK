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

#include <glog/logging.h>

#include "mynteye/api.h"

#include "data/pc_viewer.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);

  api->EnableStreamData(Stream::POINTS);

  api->Start(Source::VIDEO_STREAMING);

  cv::namedWindow("frame");
  PCViewer pcviewer;

  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);

    cv::Mat img;
    cv::hconcat(left_data.frame, right_data.frame, img);
    cv::imshow("frame", img);

    auto &&points_data = api->GetStreamData(Stream::POINTS);
    if (!points_data.frame.empty()) {
      pcviewer.Update(points_data.frame);
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
    if (pcviewer.WasStopped()) {
      break;
    }
  }

  api->Stop(Source::VIDEO_STREAMING);
  return 0;
}
