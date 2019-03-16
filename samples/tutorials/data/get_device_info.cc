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
#include "mynteye/api/api.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);
  if (!api) return 1;

  LOG(INFO) << "Device name: " << api->GetInfo(Info::DEVICE_NAME);
  LOG(INFO) << "Serial number: " << api->GetInfo(Info::SERIAL_NUMBER);
  LOG(INFO) << "Firmware version: " << api->GetInfo(Info::FIRMWARE_VERSION);
  LOG(INFO) << "Hardware version: " << api->GetInfo(Info::HARDWARE_VERSION);
  LOG(INFO) << "Spec version: " << api->GetInfo(Info::SPEC_VERSION);
  LOG(INFO) << "Lens type: " << api->GetInfo(Info::LENS_TYPE);
  LOG(INFO) << "IMU type: " << api->GetInfo(Info::IMU_TYPE);
  LOG(INFO) << "Nominal baseline: " << api->GetInfo(Info::NOMINAL_BASELINE);
  LOG(INFO) << "Auxiliary chip version: "
            << api->GetInfo(Info::AUXILIARY_CHIP_VERSION);
  LOG(INFO) << "Isp version: " << api->GetInfo(Info::ISP_VERSION);

  return 0;
}
