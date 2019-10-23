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
#include "mynteye/device/standard2/channels_adapter_s2.h"

#include "mynteye/logger.h"

#define DEBUG_TIME_LIMIT false

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
#if DEBUG_TIME_LIMIT
    timestamp += 4200000000;
#endif
    flag = *(data + 12);
    temperature = (*(data + 13) << 8) | *(data + 14);
    accel_or_gyro[0] = (*(data + 15) << 8) | *(data + 16);
    accel_or_gyro[1] = (*(data + 17) << 8) | *(data + 18);
    accel_or_gyro[2] = (*(data + 19) << 8) | *(data + 20);
  }
};
#pragma pack(pop)

#pragma pack(push, 1)

#define BYTE_4(data, begin) (*(data + begin) << 24) | \
                    (*(data + begin + 1) << 16) | \
                    (*(data + begin + 2) << 8) | \
                    *(data + begin + 3)
struct ImuData2 {
  std::uint32_t frame_id;
  std::uint64_t timestamp;
  std::uint8_t flag;
  float temperature;
  float accel_or_gyro[3];
  float gyro_add[3];

  ImuData2() = default;
  explicit ImuData2(const std::uint8_t *data) {
    from_data(data);
  }

  void from_data(const std::uint8_t *data) {
    std::uint32_t timestamp_l;
    std::uint32_t timestamp_h;
    frame_id = BYTE_4(data, 0);
    timestamp_h = (*(data + 4) << 24) | (*(data + 5) << 16) |
                  (*(data + 6) << 8) | *(data + 7);
    timestamp_l = (*(data + 8) << 24) | (*(data + 9) << 16) |
                  (*(data + 10) << 8) | *(data + 11);
    timestamp = (static_cast<std::uint64_t>(timestamp_h) << 32) | timestamp_l;
#if DEBUG_TIME_LIMIT
    timestamp += 4200000000;
#endif
    flag = *(data + 12);
    temperature = *((float*)(data+ 13));  // NOLINT
    // LOG(INFO) << "temperature:" << temperature;
    accel_or_gyro[0] = *((float*)(data + 17));  // NOLINT
    // LOG(INFO) << "accel_or_gyro[0]:" << accel_or_gyro[0];
    accel_or_gyro[1] = *((float*)(data + 21));  // NOLINT
    // LOG(INFO) << "accel_or_gyro[1]:" << accel_or_gyro[1];
    accel_or_gyro[2] = *((float*)(data + 25));  // NOLINT
    // LOG(INFO) << "accel_or_gyro[2]:" << accel_or_gyro[2];
    if (flag == 3) {
      gyro_add[0] = *((float*)(data + 29));  // NOLINT
      gyro_add[1] = *((float*)(data + 33));  // NOLINT
      gyro_add[2] = *((float*)(data + 37));  // NOLINT
    }
  }
};

#pragma pack(pop)

void unpack_imu_segment(const ImuData &imu, ImuSegment *seg) {
  seg->frame_id = imu.frame_id;
  seg->timestamp = imu.timestamp;
  seg->flag = imu.flag & 0b0011;
  seg->is_ets = ((imu.flag & 0b0100) == 0b0100);
  seg->temperature = imu.temperature;
  seg->accel[0] = (seg->flag == 1) ? imu.accel_or_gyro[0] : 0;
  seg->accel[1] = (seg->flag == 1) ? imu.accel_or_gyro[1] : 0;
  seg->accel[2] = (seg->flag == 1) ? imu.accel_or_gyro[2] : 0;
  seg->gyro[0] = (seg->flag == 2) ? imu.accel_or_gyro[0] : 0;
  seg->gyro[1] = (seg->flag == 2) ? imu.accel_or_gyro[1] : 0;
  seg->gyro[2] = (seg->flag == 2) ? imu.accel_or_gyro[2] : 0;
}

void unpack_imu_segment2(const ImuData2 &imu, ImuSegment2 *seg) {
  seg->frame_id = imu.frame_id;
  seg->timestamp = imu.timestamp;
  seg->flag = imu.flag & 0b0011;
  seg->is_ets = ((imu.flag & 0b0100) == 0b0100);
  seg->temperature = imu.temperature;
  if (seg->flag == 1) {
    seg->accel[0] = imu.accel_or_gyro[0];
    seg->accel[1] = imu.accel_or_gyro[1];
    seg->accel[2] = imu.accel_or_gyro[2];
    seg->gyro[0] = 0.;
    seg->gyro[1] = 0.;
    seg->gyro[2] = 0.;
  } else if (seg->flag == 2) {
    seg->gyro[0] = imu.accel_or_gyro[0];
    seg->gyro[1] = imu.accel_or_gyro[1];
    seg->gyro[2] = imu.accel_or_gyro[2];
    seg->accel[0] = 0.;
    seg->accel[1] = 0.;
    seg->accel[2] = 0.;
  } else if (seg->flag == 3) {
    seg->gyro[0] = imu.accel_or_gyro[0];
    seg->gyro[1] = imu.accel_or_gyro[1];
    seg->gyro[2] = imu.accel_or_gyro[2];
    seg->accel[0] = imu.gyro_add[0];
    seg->accel[1] = imu.gyro_add[1];
    seg->accel[2] = imu.gyro_add[2];
  }
}

void unpack_imu_packet(const std::uint8_t *data, ImuPacket *pkg) {
  std::size_t data_n = sizeof(ImuData);  // 21
  for (std::size_t i = 0; i < pkg->count; i++) {
    ImuSegment seg;
    unpack_imu_segment(ImuData(data + data_n * i), &seg);
    pkg->segments.push_back(seg);
  }
  if (pkg->count) {
    pkg->serial_number = pkg->segments.back().frame_id;
  } else {
    LOG(ERROR) << "The imu data pipeline lost more than 5 samples continuously, "
               << "please check the device and firmware";
  }
}

void unpack_imu_packet2(
    const std::uint8_t *data, ImuPacket2 *pkg, bool is_correspondence_on) {
  std::size_t data_n = 29;
  if (is_correspondence_on) {
    data_n = 41;
  }
  for (std::size_t i = 0; i < pkg->count; i++) {
    ImuSegment2 seg;
    unpack_imu_segment2(ImuData2(data + data_n * i), &seg);
    pkg->segments.push_back(seg);
  }
  if (pkg->count) {
    pkg->serial_number = pkg->segments.back().frame_id;
  } else {
    LOG(ERROR) << "The imu data pipeline lost more than 5 samples continuously, "
               << "please check the device and firmware";
  }
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

void unpack_imu_res_packet2(
    const std::uint8_t *data, ImuResPacket2 *res, bool is_correspondence_on) {
  res->header = *data;
  // u_int64_t* jj = (u_int64_t*) data;
  res->state = *(data + 1);
  res->size = (*(data + 2) << 8) | *(data + 3);
  std::size_t data_n = 29;
  if (is_correspondence_on) {
    data_n = 41;
  }
  // LOG(INFO) << "size:" << data_n;
  ImuPacket2 packet;
  packet.count = res->size / data_n;
  unpack_imu_packet2(data + 4, &packet, is_correspondence_on);
  res->packets.push_back(packet);
  res->checksum = *(data + 4 + res->size);
}

}  // namespace

Standard2ChannelsAdapter::Standard2ChannelsAdapter(const Model &model)
  : ChannelsAdapter(model) {
}

Standard2ChannelsAdapter::~Standard2ChannelsAdapter() {
}

std::int32_t Standard2ChannelsAdapter::GetAccelRangeDefault() {
  return 12;
}

std::vector<std::int32_t> Standard2ChannelsAdapter::GetAccelRangeValues() {
  return {6, 12, 24, 48};
}

std::int32_t Standard2ChannelsAdapter::GetGyroRangeDefault() {
  return 1000;
}

std::vector<std::int32_t> Standard2ChannelsAdapter::GetGyroRangeValues() {
  return {250, 500, 1000, 2000, 4000};
}

void Standard2ChannelsAdapter::GetImuResPacket(
    const std::uint8_t *data, ImuResPacket *res) {
  unpack_imu_res_packet(data, res);
}

void Standard2ChannelsAdapter::GetImuResPacket2(
    const std::uint8_t *data, ImuResPacket2 *res, bool is_correspondence_on) {
  unpack_imu_res_packet2(data, res, is_correspondence_on);
}

MYNTEYE_END_NAMESPACE
