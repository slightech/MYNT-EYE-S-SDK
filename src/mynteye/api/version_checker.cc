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
#include "mynteye/api/version_checker.h"
#include "mynteye/device/utils.h"
#include "mynteye/logger.h"
#include "mynteye/types.h"

MYNTEYE_BEGIN_NAMESPACE

typedef struct {
    const std::string device_type;
    const std::string sdk_version;
    const std::string firmware_version;
}firmware_version_match_table_unit;

/** firmware/sdk version matched table */
static const firmware_version_match_table_unit FSVM_TABLE[] ={
    {"MYNT-EYE-S1030",    "2.3.1.0",    "2.0.0"},
    {"MYNT-EYE-S1030",    "2.3.1.0",    "2.0.0"},
    {}
};

bool checkFirmwareVersion(const std::shared_ptr<API> api) {
  LOG(INFO) << "SDK version: " << api->GetInfo(Info::SDK_VERSION);
  LOG(INFO) << "Device name: " << api->GetInfo(Info::DEVICE_NAME);
  LOG(INFO) << "Serial number: " << api->GetInfo(Info::SERIAL_NUMBER);
  LOG(INFO) << "Firmware version: " << api->GetInfo(Info::FIRMWARE_VERSION);
  LOG(INFO) << "Hardware version: " << api->GetInfo(Info::HARDWARE_VERSION);
  LOG(INFO) << "Spec version: " << api->GetInfo(Info::SPEC_VERSION);
  LOG(INFO) << "Lens type: " << api->GetInfo(Info::LENS_TYPE);
  LOG(INFO) << "IMU type: " << api->GetInfo(Info::IMU_TYPE);
  LOG(INFO) << "Nominal baseline: " << api->GetInfo(Info::NOMINAL_BASELINE);
  return true;
}

MYNTEYE_END_NAMESPACE

