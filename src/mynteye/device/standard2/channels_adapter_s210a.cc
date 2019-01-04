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
#include "mynteye/device/standard2/channels_adapter_s210a.h"

#include "mynteye/device/config.h"
#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

namespace {

#pragma pack(push, 1)
struct ImuData {
  std::uint32_t frame_id;
  std::uint64_t timestamp;
  std::uint8_t flag;
  std::int16_t temperature;
  std::int16_t accel_or_gyro[3];

  ImuData() = default;
  explicit ImuData(const std::uint8_t *data) {
    from_data(data);
  }

  void from_data(const std::uint8_t *data) {
    std::uint32_t timestamp_l;
    std::uint32_t timestamp_h;

    frame_id = (*(data) << 24) | (*(data + 1) << 16) | (*(data + 2) << 8) |
                    *(data + 3);
    timestamp_h = (*(data + 4) << 24) | (*(data + 5) << 16) |
                  (*(data + 6) << 8) | *(data + 7);
    timestamp_l = (*(data + 8) << 24) | (*(data + 9) << 16) |
                  (*(data + 10) << 8) | *(data + 11);
    timestamp = (static_cast<std::uint64_t>(timestamp_h) << 32) | timestamp_l;
    flag = *(data + 12);
    temperature = (*(data + 13) << 8) | *(data + 14);
    accel_or_gyro[0] = (*(data + 15) << 8) | *(data + 16);
    accel_or_gyro[1] = (*(data + 17) << 8) | *(data + 18);
    accel_or_gyro[2] = (*(data + 19) << 8) | *(data + 20);
  }
};
#pragma pack(pop)

void unpack_imu_segment(const ImuData &imu, ImuSegment *seg) {
  seg->frame_id = imu.frame_id;
  seg->timestamp = imu.timestamp;
  seg->flag = imu.flag;
  seg->temperature = imu.temperature;
  seg->accel[0] = (seg->flag == 1) ? imu.accel_or_gyro[0] : 0;
  seg->accel[1] = (seg->flag == 1) ? imu.accel_or_gyro[1] : 0;
  seg->accel[2] = (seg->flag == 1) ? imu.accel_or_gyro[2] : 0;
  seg->gyro[0] = (seg->flag == 2) ? imu.accel_or_gyro[0] : 0;
  seg->gyro[1] = (seg->flag == 2) ? imu.accel_or_gyro[1] : 0;
  seg->gyro[2] = (seg->flag == 2) ? imu.accel_or_gyro[2] : 0;
}

void unpack_imu_packet(const std::uint8_t *data, ImuPacket *pkg) {
  std::size_t data_n = sizeof(ImuData);  // 21
  for (std::size_t i = 0; i < pkg->count; i++) {
    ImuSegment seg;
    unpack_imu_segment(ImuData(data + data_n * i), &seg);
    pkg->segments.push_back(seg);
  }
  pkg->serial_number = pkg->segments.back().frame_id;
}

void unpack_imu_res_packet(const std::uint8_t *data, ImuResPacket *res) {
  res->header = *data;
  res->state = *(data + 1);
  res->size = (*(data + 2) << 8) | *(data + 3);

  std::size_t data_n = sizeof(ImuData);  // 21
  ImuPacket packet;
  packet.count = res->size / data_n;
  unpack_imu_packet(data + 4, &packet);
  res->packets.push_back(packet);
  res->checksum = *(data + 4 + res->size);
}

}  // namespace

Standard210aChannelsAdapter::Standard210aChannelsAdapter() {
}

Standard210aChannelsAdapter::~Standard210aChannelsAdapter() {
}

std::set<Option> Standard210aChannelsAdapter::GetOptionSupports() {
  return option_supports_map.at(Model::STANDARD210A);
}

std::int32_t Standard210aChannelsAdapter::GetAccelRangeDefault() {
  return 12;
}

std::vector<std::int32_t> Standard210aChannelsAdapter::GetAccelRangeValues() {
  return {6, 12, 24, 48};
}

std::int32_t Standard210aChannelsAdapter::GetGyroRangeDefault() {
  return 1000;
}

std::vector<std::int32_t> Standard210aChannelsAdapter::GetGyroRangeValues() {
  return {250, 500, 1000, 2000, 4000};
}

void Standard210aChannelsAdapter::GetImuResPacket(
    const std::uint8_t *data, ImuResPacket *res) {
  unpack_imu_res_packet(data, res);
}

std::size_t Standard210aChannelsAdapter::GetImgParamsFromData(
    const std::uint8_t *data, const Version *version,
    Channels::img_params_t *img_params) {
  std::size_t i = 0;

  Extrinsics ex_right_to_left;
  {
    auto in_left = std::make_shared<IntrinsicsPinhole>();
    auto in_right = std::make_shared<IntrinsicsPinhole>();
    i += bytes::from_data(in_left.get(), data + i, version);
    i += bytes::from_data(in_right.get(), data + i, version);
    (*img_params)[{1280, 400}] = {true, in_left, in_right, ex_right_to_left};
  }
  {
    auto in_left = std::make_shared<IntrinsicsPinhole>();
    auto in_right = std::make_shared<IntrinsicsPinhole>();
    i += bytes::from_data(in_left.get(), data + i, version);
    i += bytes::from_data(in_right.get(), data + i, version);
    (*img_params)[{2560, 800}] = {true, in_left, in_right, ex_right_to_left};
  }
  {
    i += bytes::from_data(&ex_right_to_left, data + i, version);
    (*img_params)[{1280, 400}].ex_right_to_left = ex_right_to_left;
    (*img_params)[{2560, 800}].ex_right_to_left = ex_right_to_left;
  }

  return i;
}

std::size_t Standard210aChannelsAdapter::SetImgParamsToData(
    const Channels::img_params_t *img_params, const Version *version,
    std::uint8_t *data) {
  std::size_t i = 3;  // skip id, size

  {
    auto params = (*img_params).at({1280, 400});
    auto in_left = std::dynamic_pointer_cast<IntrinsicsPinhole>(params.in_left);
    auto in_right = std::dynamic_pointer_cast<IntrinsicsPinhole>(
        params.in_right);
    i += bytes::to_data(in_left.get(), data + i, version);
    i += bytes::to_data(in_right.get(), data + i, version);
  }
  {
    auto params = (*img_params).at({2560, 800});
    auto in_left = std::dynamic_pointer_cast<IntrinsicsPinhole>(params.in_left);
    auto in_right = std::dynamic_pointer_cast<IntrinsicsPinhole>(
        params.in_right);
    i += bytes::to_data(in_left.get(), data + i, version);
    i += bytes::to_data(in_right.get(), data + i, version);
    i += bytes::to_data(&params.ex_right_to_left, data + i, version);
  }

  // others
  std::size_t size = i - 3;
  data[0] = Channels::FID_IMG_PARAMS;
  data[1] = static_cast<std::uint8_t>((size >> 8) & 0xFF);
  data[2] = static_cast<std::uint8_t>(size & 0xFF);
  return size + 3;
}

MYNTEYE_END_NAMESPACE
