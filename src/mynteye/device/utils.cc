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
#include "mynteye/device/utils.h"

#include <algorithm>
#include <cstdlib>

#include "mynteye/logger.h"

#include "mynteye/device/context.h"
#include "mynteye/device/device.h"

MYNTEYE_BEGIN_NAMESPACE

bool sort_sn(std::shared_ptr<Device> device1,
    std::shared_ptr<Device> device2) {
  return device1->GetInfo(Info::SERIAL_NUMBER) <
    device2->GetInfo(Info::SERIAL_NUMBER);
}

namespace device {

std::shared_ptr<Device> select() {
  LOG(INFO) << "Detecting MYNT EYE devices";
  Context context;
  auto &&devices = context.devices();

  std::size_t n = devices.size();
  if (n <= 0) {
    LOG(ERROR) << "No MYNT EYE devices :(";
    return nullptr;
  }

  if (n > 1)
    sort(devices.begin(), devices.end(), sort_sn);

  LOG(INFO) << "MYNT EYE devices:";
  for (std::size_t i = 0; i < n; i++) {
    auto &&device = devices[i];
    LOG(INFO) << "  index: " << i
              << ", name: " << device->GetInfo(Info::DEVICE_NAME)
              << ", sn: " << device->GetInfo(Info::SERIAL_NUMBER)
              << ", firmware: " << device->GetInfo(Info::FIRMWARE_VERSION);
  }

  std::shared_ptr<Device> device = nullptr;
  if (n <= 1) {
    device = devices[0];
    LOG(INFO) << "Only one MYNT EYE device, select index: 0";
  } else {
    while (true) {
      std::size_t i;
      LOG(INFO) << "There are " << n << " MYNT EYE devices, select index: ";
      std::cin >> i;
      if (i >= n) {
        LOG(WARNING) << "Index out of range :(";
        continue;
      }
      device = devices[i];
      break;
    }
  }

  return device;
}

MYNTEYE_NAMESPACE::StreamRequest select_request(
    const std::shared_ptr<Device> &device, bool *ok) {
  auto &&requests = device->GetStreamRequests();
  std::size_t n = requests.size();
  // TODO(Kalman): Get request size by uvc enum
  if (device->GetModel() == Model::STANDARD &&
        device->GetInfo()->firmware_version < Version(2, 4)) {
    n -= 1;
  }
  if (n <= 0) {
    LOG(ERROR) << "No MYNT EYE devices :(";
    *ok = false;
    return {};
  }
  auto requests_display = requests;
  if (Model::STANDARD == device->GetModel()) {
    for (std::size_t i = 0; i < n; i++) {
      requests_display[i].fps = 25;
    }
  }

  LOG(INFO) << "MYNT EYE requests:";
  for (std::size_t i = 0; i < n; i++) {
    auto &&request = requests_display[i];
    LOG(INFO) << "  index: " << i
              << ", request: " << request;
  }

  if (n <= 1) {
    LOG(INFO) << "Only one stream request, select index: 0";
    *ok = true;
    return requests[0];
  } else {
    while (true) {
      std::size_t i;
      LOG(INFO) << "There are " << n << " stream requests, select index: ";
      std::cin >> i;
      if (i >= n) {
        LOG(WARNING) << "Index out of range :(";
        continue;
      }
      *ok = true;
      return requests[i];
    }
  }
}

}  // namespace device

namespace utils {

float get_real_exposure_time(
    std::int32_t frame_rate, std::uint16_t exposure_time) {
  float real_max = 0;
  switch (frame_rate) {
    case 10:
      real_max = 18;
      break;
    case 15:
      real_max = 18;
      break;
    case 20:
      real_max = 18;
      break;
    case 25:
      real_max = 18;
      break;
    case 30:
      real_max = 18;
      break;
    case 35:
      real_max = 18;
      break;
    case 40:
      real_max = 18;
      break;
    case 45:
      real_max = 18;
      break;
    case 50:
      real_max = 17;
      break;
    case 55:
      real_max = 16.325;
      break;
    case 60:
      real_max = 15;
      break;
    default:
      LOG(ERROR) << "Invalid frame rate: " << frame_rate;
      return exposure_time;
  }
  return exposure_time * real_max / 480.f;
}

std::string get_sdk_root_dir() {
  if (const char* root = std::getenv("MYNTEYES_SDK_ROOT")) {
    // LOG(INFO) << "Environment variable MYNTEYES_SDK_ROOT found: " << root;
    return std::string(root);
  } else {
    return std::string(MYNTEYE_SDK_ROOT_DIR);
  }
}

std::string get_sdk_install_dir() {
  if (const char* root = std::getenv("MYNTEYES_SDK_ROOT")) {
    // LOG(INFO) << "Environment variable MYNTEYES_SDK_ROOT found: " << root;
    return std::string(root);
  } else {
    return std::string(MYNTEYE_SDK_INSTALL_DIR);
  }
}

}  // namespace utils

MYNTEYE_END_NAMESPACE
