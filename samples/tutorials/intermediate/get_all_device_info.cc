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
#include "mynteye/logger.h"
#include "mynteye/device/context.h"
#include "mynteye/device/device.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  glog_init _(argc, argv);

  LOG(INFO) << "Detecting MYNT EYE devices";
  Context context;
  auto &&devices = context.devices();

  size_t n = devices.size();
  if (n <= 0) {
    LOG(ERROR) << "No MYNT EYE devices :(";
    return 1;
  }

  for (size_t i = 0; i < n; i++) {
    auto &&device = devices[i];
    LOG(INFO) << "MYNT EYE device index: " << i;
    LOG(INFO) << "  Device name: " << device->GetInfo(Info::DEVICE_NAME);
    LOG(INFO) << "  Serial number: " << device->GetInfo(Info::SERIAL_NUMBER);
    LOG(INFO) << "  Firmware version: "
              << device->GetInfo(Info::FIRMWARE_VERSION);
    LOG(INFO) << "  Hardware version: "
              << device->GetInfo(Info::HARDWARE_VERSION);
    LOG(INFO) << "  Spec version: " << device->GetInfo(Info::SPEC_VERSION);
    LOG(INFO) << "  Lens type: " << device->GetInfo(Info::LENS_TYPE);
    LOG(INFO) << "  IMU type: " << device->GetInfo(Info::IMU_TYPE);
    LOG(INFO) << "  Nominal baseline: "
              << device->GetInfo(Info::NOMINAL_BASELINE);
  }

  return 0;
}
