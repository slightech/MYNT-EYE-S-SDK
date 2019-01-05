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
#include "mynteye/device/channel/file_channel.h"

#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

// FileChannel

FileChannel::FileChannel() {
  dev_info_parser_ = std::make_shared<DeviceInfoParser>();
  img_params_parser_ = std::make_shared<ImgParamsParser>();
  imu_params_parser_ = std::make_shared<ImuParamsParser>();
}

FileChannel::~FileChannel() {
}

std::size_t FileChannel::GetDeviceInfoFromData(
    const std::uint8_t *data, device_info_t *info) {
  auto n = dev_info_parser_->GetFromData(data, info);
  auto spec_version = info->spec_version;
  img_params_parser_->SetSpecVersion(spec_version);
  imu_params_parser_->SetSpecVersion(spec_version);
  return n;
}

std::size_t FileChannel::SetDeviceInfoToData(
    const device_info_t *info, std::uint8_t *data) {
  auto spec_version = info->spec_version;
  img_params_parser_->SetSpecVersion(spec_version);
  imu_params_parser_->SetSpecVersion(spec_version);
  return dev_info_parser_->SetToData(info, data);
}

std::size_t FileChannel::GetImgParamsFromData(
    const std::uint8_t *data, img_params_t *img_params) {
  CHECK_NOTNULL(img_params_parser_);
  return img_params_parser_->GetFromData(data, img_params);
}

std::size_t FileChannel::SetImgParamsToData(
    const img_params_t *img_params, std::uint8_t *data) {
  CHECK_NOTNULL(img_params_parser_);
  return img_params_parser_->SetToData(img_params, data);
}

std::size_t FileChannel::GetImuParamsFromData(
    const std::uint8_t *data, imu_params_t *imu_params) {
  return imu_params_parser_->GetFromData(data, imu_params);
}

std::size_t FileChannel::SetImuParamsToData(
    const imu_params_t *imu_params, std::uint8_t *data) {
  return imu_params_parser_->SetToData(imu_params, data);
}

// DeviceInfoParser

DeviceInfoParser::DeviceInfoParser() {
}

DeviceInfoParser::~DeviceInfoParser() {
}

std::size_t DeviceInfoParser::GetFromData(
    const std::uint8_t *data, device_info_t *info) const {
  std::size_t i = 4;  // skip vid, pid
  // name, 16
  info->name = bytes::_from_data(data + i, 16);
  i += 16;
  // serial_number, 16
  info->serial_number = bytes::_from_data(data + i, 16);
  i += 16;
  // firmware_version, 2
  info->firmware_version.set_major(data[i]);
  info->firmware_version.set_minor(data[i + 1]);
  i += 2;
  // hardware_version, 3
  info->hardware_version.set_major(data[i]);
  info->hardware_version.set_minor(data[i + 1]);
  info->hardware_version.set_flag(std::bitset<8>(data[i + 2]));
  i += 3;
  // spec_version, 2
  info->spec_version.set_major(data[i]);
  info->spec_version.set_minor(data[i + 1]);
  i += 2;
  // lens_type, 4
  info->lens_type.set_vendor(bytes::_from_data<std::uint16_t>(data + i));
  info->lens_type.set_product(bytes::_from_data<std::uint16_t>(data + i + 2));
  i += 4;
  // imu_type, 4
  info->imu_type.set_vendor(bytes::_from_data<std::uint16_t>(data + i));
  info->imu_type.set_product(bytes::_from_data<std::uint16_t>(data + i + 2));
  i += 4;
  // nominal_baseline, 2
  info->nominal_baseline = bytes::_from_data<std::uint16_t>(data + i);
  i += 2;

  // get other infos according to spec_version

  return i;
}

std::size_t DeviceInfoParser::SetToData(
    const device_info_t *info, std::uint8_t *data) const {
  std::size_t i = 3;  // skip id, size
  i += 4;             // skip vid, pid
  // name, 16
  bytes::_to_data(info->name, data + i, 16);
  i += 16;
  // serial_number, 16
  bytes::_to_data(info->serial_number, data + i, 16);
  i += 16;
  // firmware_version, 2
  data[i] = info->firmware_version.major();
  data[i + 1] = info->firmware_version.minor();
  i += 2;
  // hardware_version, 3
  data[i] = info->hardware_version.major();
  data[i + 1] = info->hardware_version.minor();
  data[i + 2] =
      static_cast<std::uint8_t>(info->hardware_version.flag().to_ulong());
  i += 3;
  // spec_version, 2
  data[i] = info->spec_version.major();
  data[i + 1] = info->spec_version.minor();
  i += 2;
  // lens_type, 4
  bytes::_to_data(info->lens_type.vendor(), data + i);
  bytes::_to_data(info->lens_type.product(), data + i + 2);
  i += 4;
  // imu_type, 4
  bytes::_to_data(info->imu_type.vendor(), data + i);
  bytes::_to_data(info->imu_type.product(), data + i + 2);
  i += 4;
  // nominal_baseline, 2
  bytes::_to_data(info->nominal_baseline, data + i);
  i += 2;

  // set other infos according to spec_version

  // others
  std::size_t size = i - 3;
  data[0] = FID_DEVICE_INFO;
  data[1] = static_cast<std::uint8_t>((size >> 8) & 0xFF);
  data[2] = static_cast<std::uint8_t>(size & 0xFF);
  return size + 3;
}

// ImgParamsParser

ImgParamsParser::ImgParamsParser() {
}

ImgParamsParser::~ImgParamsParser() {
}

std::size_t ImgParamsParser::GetFromData(
    const std::uint8_t *data, img_params_t *img_params) const {
  if (spec_version_ == Version(1, 0) || spec_version_ == Version(1, 1)) {
    // get img params without version header
    if (spec_version_ == Version(1, 0)) {
      return GetFromData_v1_0(data, img_params);
    } else {
      return GetFromData_v1_1(data, img_params);
    }
  } else {
    // get img params with version header
    return GetFromData_new(data, img_params);
  }
}

std::size_t ImgParamsParser::SetToData(
    const img_params_t *img_params, std::uint8_t *data) const {
  if (spec_version_ == Version(1, 0) || spec_version_ == Version(1, 1)) {
    // set img params without version header
    if (spec_version_ == Version(1, 0)) {
      return SetToData_v1_0(img_params, data);
    } else {
      return SetToData_v1_1(img_params, data);
    }
  } else {
    // set img params with version header
    return SetToData_new(img_params, data);
  }
}

std::size_t ImgParamsParser::GetFromData_v1_0(
    const std::uint8_t *data, img_params_t *img_params) const {
  std::size_t i = 0;

  auto in_left = std::make_shared<IntrinsicsPinhole>();
  auto in_right = std::make_shared<IntrinsicsPinhole>();
  Extrinsics ex_right_to_left;

  i += bytes::from_data(in_left.get(), data + i);
  i += bytes::from_data(in_right.get(), data + i);
  i += bytes::from_data(&ex_right_to_left, data + i);
  (*img_params)[{752, 480}] = {true, spec_version_.to_string(),
      in_left, in_right, ex_right_to_left};

  return i;
}

std::size_t ImgParamsParser::SetToData_v1_0(
    const img_params_t *img_params, std::uint8_t *data) const {
  std::size_t i = 3;  // skip id, size

  auto params = (*img_params).at({752, 480});
  auto in_left = std::dynamic_pointer_cast<IntrinsicsPinhole>(params.in_left);
  auto in_right = std::dynamic_pointer_cast<IntrinsicsPinhole>(params.in_right);
  i += bytes::to_data(in_left.get(), data + i);
  i += bytes::to_data(in_right.get(), data + i);
  i += bytes::to_data(&params.ex_right_to_left, data + i);

  // others
  std::size_t size = i - 3;
  data[0] = FID_IMG_PARAMS;
  data[1] = static_cast<std::uint8_t>((size >> 8) & 0xFF);
  data[2] = static_cast<std::uint8_t>(size & 0xFF);
  return size + 3;
}

std::size_t ImgParamsParser::GetFromData_v1_1(
    const std::uint8_t *data, img_params_t *img_params) const {
  std::size_t i = 0;

  Extrinsics ex_right_to_left;
  {
    auto in_left = std::make_shared<IntrinsicsPinhole>();
    auto in_right = std::make_shared<IntrinsicsPinhole>();
    i += bytes::from_data(in_left.get(), data + i);
    i += bytes::from_data(in_right.get(), data + i);
    (*img_params)[{1280, 400}] = {true, spec_version_.to_string(),
        in_left, in_right, ex_right_to_left};
  }
  {
    auto in_left = std::make_shared<IntrinsicsPinhole>();
    auto in_right = std::make_shared<IntrinsicsPinhole>();
    i += bytes::from_data(in_left.get(), data + i);
    i += bytes::from_data(in_right.get(), data + i);
    (*img_params)[{2560, 800}] = {true, spec_version_.to_string(),
        in_left, in_right, ex_right_to_left};
  }
  {
    i += bytes::from_data(&ex_right_to_left, data + i);
    (*img_params)[{1280, 400}].ex_right_to_left = ex_right_to_left;
    (*img_params)[{2560, 800}].ex_right_to_left = ex_right_to_left;
  }

  return i;
}

std::size_t ImgParamsParser::SetToData_v1_1(
    const img_params_t *img_params, std::uint8_t *data) const {
  std::size_t i = 3;  // skip id, size

  {
    auto params = (*img_params).at({1280, 400});
    auto in_left = std::dynamic_pointer_cast<IntrinsicsPinhole>(params.in_left);
    auto in_right = std::dynamic_pointer_cast<IntrinsicsPinhole>(
        params.in_right);
    i += bytes::to_data(in_left.get(), data + i);
    i += bytes::to_data(in_right.get(), data + i);
  }
  {
    auto params = (*img_params).at({2560, 800});
    auto in_left = std::dynamic_pointer_cast<IntrinsicsPinhole>(params.in_left);
    auto in_right = std::dynamic_pointer_cast<IntrinsicsPinhole>(
        params.in_right);
    i += bytes::to_data(in_left.get(), data + i);
    i += bytes::to_data(in_right.get(), data + i);
    i += bytes::to_data(&params.ex_right_to_left, data + i);
  }

  // others
  std::size_t size = i - 3;
  data[0] = FID_IMG_PARAMS;
  data[1] = static_cast<std::uint8_t>((size >> 8) & 0xFF);
  data[2] = static_cast<std::uint8_t>(size & 0xFF);
  return size + 3;
}

std::size_t ImgParamsParser::GetFromData_new(
    const std::uint8_t *data, img_params_t *img_params) const {
  return 0;
}
std::size_t ImgParamsParser::SetToData_new(
    const img_params_t *img_params, std::uint8_t *data) const {
  return 0;
}

// ImuParamsParser

ImuParamsParser::ImuParamsParser() {
}

ImuParamsParser::~ImuParamsParser() {
}

std::size_t ImuParamsParser::GetFromData(
    const std::uint8_t *data, imu_params_t *imu_params) const {
  if (spec_version_ == Version(1, 0) || spec_version_ == Version(1, 1)) {
    // get imu params without version header
    return GetFromData_old(data, imu_params);
  } else {
    // get imu params with version header
    return GetFromData_new(data, imu_params);
  }
}

std::size_t ImuParamsParser::SetToData(
    const imu_params_t *imu_params, std::uint8_t *data) const {
  if (spec_version_ == Version(1, 0) || spec_version_ == Version(1, 1)) {
    // set imu params without version header
    return SetToData_old(imu_params, data);
  } else {
    // set imu params with version header
    return SetToData_new(imu_params, data);
  }
}

std::size_t ImuParamsParser::GetFromData_old(
    const std::uint8_t *data, imu_params_t *imu_params) const {
  std::size_t i = 0;
  i += bytes::from_data(&imu_params->in_accel, data + i);
  i += bytes::from_data(&imu_params->in_gyro, data + i);
  i += bytes::from_data(&imu_params->ex_left_to_imu, data + i);
  return i;
}

std::size_t ImuParamsParser::SetToData_old(
    const imu_params_t *imu_params, std::uint8_t *data) const {
  std::size_t i = 3;  // skip id, size
  i += bytes::to_data(&imu_params->in_accel, data + i);
  i += bytes::to_data(&imu_params->in_gyro, data + i);
  i += bytes::to_data(&imu_params->ex_left_to_imu, data + i);
  // others
  std::size_t size = i - 3;
  data[0] = FID_IMU_PARAMS;
  data[1] = static_cast<std::uint8_t>((size >> 8) & 0xFF);
  data[2] = static_cast<std::uint8_t>(size & 0xFF);
  return size + 3;
}

std::size_t ImuParamsParser::GetFromData_new(
    const std::uint8_t *data, imu_params_t *imu_params) const {
  std::size_t i = 0;
  // version, 2
  Version version(data[i], data[i + 1]);
  imu_params->version = version.to_string();
  i += 2;
  // get imu params according to version
  if (version == Version(1, 2)) {  // v1.2
    i += bytes::from_data(&imu_params->in_accel, data + i);
    i += bytes::from_data(&imu_params->in_gyro, data + i);
    i += bytes::from_data(&imu_params->ex_left_to_imu, data + i);
  } else {
    LOG(FATAL) << "Could not get imu params of version "
        << version.to_string() << ", please use latest SDK.";
  }
  return i;
}

std::size_t ImuParamsParser::SetToData_new(
    const imu_params_t *imu_params, std::uint8_t *data) const {
  std::size_t i = 3;  // skip id, size
  // version, 2
  Version version(imu_params->version);
  data[i] = version.major();
  data[i + 1] = version.minor();
  i += 2;
  // set imu params according to version
  if (version == Version(1, 2)) {  // v1.2
    i += bytes::to_data(&imu_params->in_accel, data + i);
    i += bytes::to_data(&imu_params->in_gyro, data + i);
    i += bytes::to_data(&imu_params->ex_left_to_imu, data + i);
  } else {
    LOG(FATAL) << "Could not set imu params of version "
        << version.to_string() << ", please use latest SDK.";
  }
  // others
  std::size_t size = i - 3;
  data[0] = FID_IMU_PARAMS;
  data[1] = static_cast<std::uint8_t>((size >> 8) & 0xFF);
  data[2] = static_cast<std::uint8_t>(size & 0xFF);
  return size + 3;
}

MYNTEYE_END_NAMESPACE
