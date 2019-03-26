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
#ifndef MYNTEYE_DEVICE_CHANNEL_FILE_CHANNEL_H_
#define MYNTEYE_DEVICE_CHANNEL_FILE_CHANNEL_H_
#pragma once

#include <map>
#include <memory>

#include "mynteye/mynteye.h"
#include "mynteye/device/device.h"
#include "mynteye/device/channel/bytes.h"

MYNTEYE_BEGIN_NAMESPACE

class DeviceInfoParser;
class ImgParamsParser;
class ImuParamsParser;

class FileChannel {
 public:
  using device_info_t = DeviceInfo;

  using img_params_t = std::map<Resolution, device::img_params_t>;
  using imu_params_t = device::imu_params_t;

  FileChannel();
  ~FileChannel();

  std::size_t GetDeviceInfoFromData(
      const std::uint8_t *data, const std::uint16_t &data_size,
      device_info_t *info);
  std::size_t SetDeviceInfoToData(
      const device_info_t *info, std::uint8_t *data);

  std::size_t GetImgParamsFromData(
      const std::uint8_t *data, const std::uint16_t &data_size,
      img_params_t *img_params);
  std::size_t SetImgParamsToData(
      const img_params_t *img_params, std::uint8_t *data);

  std::size_t GetImuParamsFromData(
      const std::uint8_t *data, const std::uint16_t &data_size,
      imu_params_t *imu_params);
  std::size_t SetImuParamsToData(
      const imu_params_t *imu_params, std::uint8_t *data);

 private:
  std::shared_ptr<DeviceInfoParser> dev_info_parser_;
  std::shared_ptr<ImgParamsParser> img_params_parser_;
  std::shared_ptr<ImuParamsParser> imu_params_parser_;
};

class DeviceInfoParser {
 public:
  using device_info_t = FileChannel::device_info_t;

  DeviceInfoParser();
  ~DeviceInfoParser();

  std::size_t GetFromData(
      const std::uint8_t *data, const std::uint16_t &data_size,
      device_info_t *info) const;
  std::size_t SetToData(
      const device_info_t *info, std::uint8_t *data) const;
};

class ImgParamsParser {
 public:
  using img_params_t = FileChannel::img_params_t;

  ImgParamsParser();
  ~ImgParamsParser();

  void SetSpecVersion(const Version& spec_version) {
    spec_version_ = spec_version;
  }

  std::size_t GetFromData(
      const std::uint8_t *data, const std::uint16_t &data_size,
      img_params_t *img_params) const;
  std::size_t SetToData(
      const img_params_t *img_params, std::uint8_t *data) const;

  std::size_t GetFromData_v1_0(
      const std::uint8_t *data, const std::uint16_t &data_size,
      img_params_t *img_params) const;
  std::size_t SetToData_v1_0(
      const img_params_t *img_params, std::uint8_t *data) const;

  std::size_t GetFromData_v1_1(
      const std::uint8_t *data, const std::uint16_t &data_size,
      img_params_t *img_params) const;
  std::size_t SetToData_v1_1(
      const img_params_t *img_params, std::uint8_t *data) const;

  std::size_t GetFromData_new(
      const std::uint8_t *data, const std::uint16_t &data_size,
      img_params_t *img_params) const;
  std::size_t SetToData_new(
      const img_params_t *img_params, std::uint8_t *data) const;

 private:
  Version spec_version_;
};

class ImuParamsParser {
 public:
  using imu_params_t = FileChannel::imu_params_t;

  ImuParamsParser();
  ~ImuParamsParser();

  void SetSpecVersion(const Version& spec_version) {
    spec_version_ = spec_version;
  }

  std::size_t GetFromData(
      const std::uint8_t *data, const std::uint16_t &data_size,
      imu_params_t *imu_params) const;
  std::size_t SetToData(
      const imu_params_t *imu_params, std::uint8_t *data) const;

  std::size_t GetFromData_old(
      const std::uint8_t *data, const std::uint16_t &data_size,
      imu_params_t *imu_params) const;
  std::size_t SetToData_old(
      const imu_params_t *imu_params, std::uint8_t *data) const;

  std::size_t GetFromData_new(
      const std::uint8_t *data, const std::uint16_t &data_size,
      imu_params_t *imu_params, const bool is_get) const;

  std::size_t SetToData_new(
      const imu_params_t *imu_params, std::uint8_t *data, const bool is_set) const;

 private:
  Version spec_version_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_CHANNEL_FILE_CHANNEL_H_
