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
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye/api/api.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);
  if (!api) return 1;

  auto request = api->GetStreamRequest();

//   struct StreamRequest {
//   /** Stream width in pixels */
//   std::uint16_t width;
//   /** Stream height in pixels */
//   std::uint16_t height;
//   /** Stream pixel format */
//   Format format;
//   /** Stream frames per second */
//   std::uint16_t fps;
//   }

//   request.fps = 10;
  api->ConfigStreamRequest(request);

  api->EnableStreamData(Stream::DEPTH);

  api->Start(Source::VIDEO_STREAMING);

  cv::namedWindow("frame");
  cv::namedWindow("depth");
  while (true) {
    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);

    if (!left_data.frame.empty() && !right_data.frame.empty()) {
      cv::Mat img;
      cv::hconcat(left_data.frame, right_data.frame, img);
      cv::imshow("frame", img);
    }

    // this code is for real depth data
    auto &&depth_data = api->GetStreamData(Stream::DEPTH);
    if (!depth_data.frame.empty()) {
      cv::imshow("depth", depth_data.frame);  // CV_16UC1
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  api->Stop(Source::VIDEO_STREAMING);
  return 0;
}
