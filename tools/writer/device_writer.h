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
#ifndef MYNTEYE_TOOLS_DEVICE_WRITER_H_  // NOLINT
#define MYNTEYE_TOOLS_DEVICE_WRITER_H_
#pragma once

#include <map>
#include <memory>
#include <string>

#include "mynteye/device/device.h"

MYNTEYE_BEGIN_NAMESPACE

class Device;

namespace tools {

class DeviceWriter {
 public:
  using dev_info_t = DeviceInfo;
  using imu_params_t = device::imu_params_t;
  using img_params_t = device::img_params_t;
  using img_params_map_t = std::map<Resolution, device::img_params_t>;

  explicit DeviceWriter(std::shared_ptr<Device> device);
  ~DeviceWriter();

  bool WriteDeviceInfo(const dev_info_t &info);
  bool WriteDeviceInfo(const std::string &filepath);

  bool WriteImgParams(const img_params_map_t &img_params_map);
  bool WriteImgParams(const std::string &filepath);

  bool WriteImuParams(const imu_params_t &params);
  bool WriteImuParams(const std::string &filepath);

  bool SaveDeviceInfo(const dev_info_t &info, const std::string &filepath);
  bool SaveImgParams(const img_params_map_t &img_params_map,
      const std::string &filepath);
  bool SaveImuParams(const imu_params_t &params, const std::string &filepath);

  /** Save all infos of this device */
  void SaveAllInfos(const std::string &dir);

 private:
  dev_info_t LoadDeviceInfo(const std::string &filepath);
  img_params_map_t LoadImgParams(const std::string &filepath);
  imu_params_t LoadImuParams(const std::string &filepath);

  std::shared_ptr<Device> device_;
};

}  // namespace tools

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_TOOLS_DEVICE_WRITER_H_ NOLINT
