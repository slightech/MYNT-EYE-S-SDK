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
#include "mynteye/device/standard/channels_adapter_s.h"

#include "mynteye/logger.h"

// for time limmit debug
#define DEBUG_TIME_LIMIT false

MYNTEYE_BEGIN_NAMESPACE

namespace {

#pragma pack(push, 1)
struct ImuData {
  std::int16_t offset;
  std::uint16_t frame_id;
  std::int16_t accel[3];
  std::int16_t temperature;
  std::int16_t gyro[3];

  ImuData() = default;
  explicit ImuData(const std::uint8_t *data) {
    from_data(data);
  }

  void from_data(const std::uint8_t *data) {
    offset = (*(data) << 8) | *(data + 1);
    frame_id = (*(data + 2) << 8) | *(data + 3);
    accel[0] = (*(data + 4) << 8) | *(data + 5);
    accel[1] = (*(data + 6) << 8) | *(data + 7);
    accel[2] = (*(data + 8) << 8) | *(data + 9);
    temperature = (*(data + 10) << 8) | *(data + 11);
    gyro[0] = (*(data + 12) << 8) | *(data + 13);
    gyro[1] = (*(data + 14) << 8) | *(data + 15);
    gyro[2] = (*(data + 16) << 8) | *(data + 17);
  }
};
#pragma pack(pop)

void unpack_imu_segment(const ImuData &imu, const std::uint32_t &timestamp,
    ImuSegment *seg) {
  seg->frame_id = static_cast<uint32_t>(imu.frame_id);
#if DEBUG_TIME_LIMIT
  std::uint32_t timestamp_test =
      timestamp + imu.offset + (std::uint32_t)4290000000;
  seg->timestamp = static_cast<uint64_t>(timestamp_test) * 10;
  LOG(WARNING) << "timestamp_out: "
               << seg->timestamp
               << "  timestamp: "
               << timestamp << "   timestamp_test: "
               << timestamp_test;
#else
  seg->timestamp = static_cast<uint64_t>(timestamp + imu.offset) * 10;
#endif
  seg->flag = 0;
  seg->temperature = imu.temperature;
  seg->accel[0] = imu.accel[0];
  seg->accel[1] = imu.accel[1];
  seg->accel[2] = imu.accel[2];
  seg->gyro[0] = imu.gyro[0];
  seg->gyro[1] = imu.gyro[1];
  seg->gyro[2] = imu.gyro[2];
}

void unpack_imu_packet(const std::uint8_t *data, ImuPacket *pkg) {
  pkg->serial_number =
      (*(data) << 24) | (*(data + 1) << 16) |
      (*(data + 2) << 8) | *(data + 3);
  std::uint32_t timestamp =
      (*(data + 4) << 24) | (*(data + 5) << 16)|
      (*(data + 6) << 8) | *(data + 7);
  pkg->count = *(data + 8);

  std::size_t data_n = sizeof(ImuData);  // 18
  for (std::size_t i = 0; i < pkg->count; i++) {
    ImuSegment seg;
    unpack_imu_segment(ImuData(data + 9 + (data_n * i)), timestamp, &seg);
    pkg->segments.push_back(seg);
  }
}

void unpack_imu_res_packet(const std::uint8_t *data, ImuResPacket *res) {
  res->header = *data;
  res->state = *(data + 1);
  res->size = (*(data + 2) << 8) | *(data + 3);

  std::size_t data_n = sizeof(ImuData);  // 18
  for (std::size_t i = 4; i < res->size;) {
    ImuPacket packet;
    unpack_imu_packet(data + i, &packet);
    res->packets.push_back(packet);
    i += 9 + (packet.count * data_n);
  }
  res->checksum = *(data + 4 + res->size);
}

}  // namespace

StandardChannelsAdapter::StandardChannelsAdapter()
  : ChannelsAdapter(Model::STANDARD) {
}

StandardChannelsAdapter::~StandardChannelsAdapter() {
}

std::int32_t StandardChannelsAdapter::GetAccelRangeDefault() {
  return 8;
}

std::vector<std::int32_t> StandardChannelsAdapter::GetAccelRangeValues() {
  return {4, 8, 16, 32};
}

std::int32_t StandardChannelsAdapter::GetGyroRangeDefault() {
  return 1000;
}

std::vector<std::int32_t> StandardChannelsAdapter::GetGyroRangeValues() {
  return {500, 1000, 2000, 4000};
}

void StandardChannelsAdapter::GetImuResPacket(
    const std::uint8_t *data, ImuResPacket *res) {
  unpack_imu_res_packet(data, res);
}

void StandardChannelsAdapter::GetImuResPacket2(
    const std::uint8_t *data, ImuResPacket2 *res, bool is_correspondence_on) {
  LOG(WARNING) << "s1 device can't use ImuResPacket2.0 check the firmware.";
}

MYNTEYE_END_NAMESPACE
