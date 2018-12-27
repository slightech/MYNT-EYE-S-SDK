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
#include <chrono>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <mutex>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye/logger.h"
#include "mynteye/mynteye.h"
#include "mynteye/types.h"
#include "mynteye/uvc/uvc.h"

struct frame {
  const void *data = nullptr;
  std::function<void()> continuation = nullptr;
  frame() {
    // VLOG(2) << __func__;
  }
  ~frame() {
    // VLOG(2) << __func__;
    data = nullptr;
    if (continuation) {
      continuation();
      continuation = nullptr;
    }
  }
};

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  glog_init _(argc, argv);

  std::vector<std::shared_ptr<uvc::device>> mynteye_devices;

  auto context = uvc::create_context();
  auto devices = uvc::query_devices(context);
  if (devices.size() <= 0) {
    LOG(ERROR) << "No devices :(";
    return 1;
  }

  for (auto &&device : devices) {
    auto vid = uvc::get_vendor_id(*device);
    // auto pid = uvc::get_product_id(*device);
    // LOG(INFO) << "vid: " << vid << ", pid: " << pid;
    if (vid == MYNTEYE_VID) {
      mynteye_devices.push_back(device);
    }
  }

  // std::string dashes(80, '-');

  size_t n = mynteye_devices.size();
  if (n <= 0) {
    LOG(ERROR) << "No MYNT EYE devices :(";
    return 1;
  }

  LOG(INFO) << "MYNT EYE devices: ";
  for (size_t i = 0; i < n; i++) {
    auto device = mynteye_devices[i];
    auto name = uvc::get_video_name(*device);
    auto vid = uvc::get_vendor_id(*device);
    auto pid = uvc::get_product_id(*device);
    LOG(INFO) << "  index: " << i << ", name: " << name << ", vid: 0x"
              << std::hex << vid << ", pid: 0x" << std::hex << pid;
  }

  std::shared_ptr<uvc::device> device = nullptr;
  if (n <= 1) {
    device = mynteye_devices[0];
    LOG(INFO) << "Only one MYNT EYE device, select index: 0";
  } else {
    while (true) {
      size_t i;
      LOG(INFO) << "There are " << n << " MYNT EYE devices, select index: ";
      std::cin >> i;
      if (i >= n) {
        LOG(WARNING) << "Index out of range :(";
        continue;
      }
      device = mynteye_devices[i];
      break;
    }
  }

  std::mutex mtx;
  std::condition_variable cv;

  std::shared_ptr<frame> frame = nullptr;
  const auto frame_ready = [&frame]() { return frame != nullptr; };
  const auto frame_empty = [&frame]() { return frame == nullptr; };

  uvc::set_device_mode(
#ifdef MYNTEYE_OS_MAC
      *device, 752, 480, static_cast<int>(Format::YUYV), 25,
#else
      *device, 1280, 400, static_cast<int>(Format::BGR888), 20,
#endif
      [&mtx, &cv, &frame, &frame_ready](
          const void *data, std::function<void()> continuation) {
        // reinterpret_cast<const std::uint8_t *>(data);
        std::unique_lock<std::mutex> lock(mtx);
        if (frame == nullptr) {
          frame = std::make_shared<struct frame>();
        } else {
          if (frame->continuation) {
            frame->continuation();
          }
        }
        frame->data = data;  // not copy here
        frame->continuation = continuation;
        if (frame_ready())
          cv.notify_one();
      });

  LOG(INFO) << "Press ESC/Q on windows to terminate";

  cv::namedWindow("frame");
  uvc::start_streaming(*device, 0);

  double t, fps = 0;
  while (true) {
    t = static_cast<double>(cv::getTickCount());

    std::unique_lock<std::mutex> lock(mtx);

    if (frame_empty()) {
      if (!cv.wait_for(lock, std::chrono::seconds(2), frame_ready))
        throw std::runtime_error("Timeout waiting for frame.");
    }

    // only lastest frame is valid
#ifdef MYNTEYE_OS_MAC
    cv::Mat img(480, 752, CV_8UC2, const_cast<void *>(frame->data));
    cv::cvtColor(img, img, cv::COLOR_YUV2BGR_YUY2);
#else
    cv::Mat img(400, 1280, CV_8UC3, const_cast<void *>(frame->data));
#endif
    cv::imshow("frame", img);

    frame = nullptr;

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }

    t = static_cast<double>(cv::getTickCount() - t);
    fps = cv::getTickFrequency() / t;
  }
  MYNTEYE_UNUSED(fps)

  uvc::stop_streaming(*device);
  // cv::destroyAllWindows();
  return 0;
}
