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
#ifndef MYNTEYE_DEVICE_CHANNEL_DEF_H_
#define MYNTEYE_DEVICE_CHANNEL_DEF_H_
#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

typedef enum Channel {
  CHANNEL_CAM_CTRL = 1,
  CHANNEL_HALF_DUPLEX = 2,
  CHANNEL_IMU_WRITE = 3,
  CHANNEL_IMU_READ = 4,
  CHANNEL_FILE = 5,
  CHANNEL_LAST
} channel_t;

typedef enum FileId {
  FID_DEVICE_INFO = 1,  // device info
  FID_IMG_PARAMS = 2,   // image intrinsics & extrinsics
  FID_IMU_PARAMS = 4,   // imu intrinsics & extrinsics
  FID_LAST,
} file_id_t;

/**
 * @ingroup datatypes
 * Imu request packet.
 */
#pragma pack(push, 1)
struct ImuReqPacket {
  std::uint8_t header;
  std::uint32_t serial_number;

  ImuReqPacket() = default;
  explicit ImuReqPacket(std::uint32_t serial_number)
      : ImuReqPacket(0x5A, serial_number) {}
  ImuReqPacket(std::uint8_t header, std::uint32_t serial_number)
      : header(header), serial_number(serial_number) {}

  std::array<std::uint8_t, 5> to_data() const {
    return {{header, static_cast<std::uint8_t>((serial_number >> 24) & 0xFF),
             static_cast<std::uint8_t>((serial_number >> 16) & 0xFF),
             static_cast<std::uint8_t>((serial_number >> 8) & 0xFF),
             static_cast<std::uint8_t>(serial_number & 0xFF)}};
  }
};
#pragma pack(pop)

/**
 * @ingroup datatypes
 * Imu request packet 2.0.
 */
#pragma pack(push, 1)
struct ImuReqPacket2 {
  std::uint8_t header;
  std::uint32_t serial_number;
  std::uint8_t correspondence_switch;

  ImuReqPacket2() = default;
  explicit ImuReqPacket2(std::uint32_t serial_number)
      : ImuReqPacket2(0x5A, serial_number) {}
  ImuReqPacket2(std::uint8_t header, std::uint32_t serial_number)
      : ImuReqPacket2(header, serial_number, false) {}
  ImuReqPacket2(std::uint8_t header,
      std::uint32_t serial_number,
      bool correspondence_switch_in)
      : header(header),
        serial_number(serial_number),
        correspondence_switch(correspondence_switch_in ? 1:0) {}

  std::array<std::uint8_t, 5> to_data() const {
    return {header, correspondence_switch, 0, 0, 0};
  }
};
#pragma pack(pop)

/**
 * @ingroup datatypes
 * Imu segment.
 */
#pragma pack(push, 1)
struct ImuSegment {
  std::uint32_t frame_id;
  std::uint64_t timestamp;
  std::uint8_t flag;
  // Is external time source
  bool is_ets;
  std::int16_t temperature;
  std::int16_t accel[3];
  std::int16_t gyro[3];
};
#pragma pack(pop)

/**
 * @ingroup datatypes
 * Imu segment.
 */
#pragma pack(push, 1)
struct ImuSegment2 {
  std::uint32_t frame_id;
  std::uint64_t timestamp;
  std::uint8_t flag;
  // Is external time source
  bool is_ets;
  float temperature;
  float accel[3];
  float gyro[3];
};
#pragma pack(pop)

/**
 * @ingroup datatypes
 * Imu packet.
 */
#pragma pack(push, 1)
struct ImuPacket {
  std::uint8_t version;
  std::uint8_t count;
  std::uint32_t serial_number;
  std::vector<ImuSegment> segments;
};
#pragma pack(pop)
/**
 * @ingroup datatypes
 * Imu packet.
 */
#pragma pack(push, 1)
struct ImuPacket2 {
  std::uint8_t version;
  std::uint8_t count;
  std::uint32_t serial_number;
  std::vector<ImuSegment2> segments;
};
#pragma pack(pop)

/**
 * @ingroup datatypes
 * Imu response packet.
 */
#pragma pack(push, 1)
struct ImuResPacket {
  std::uint8_t version;
  std::uint8_t header;
  std::uint8_t state;
  std::uint16_t size;
  std::vector<ImuPacket> packets;
  std::uint8_t checksum;
};
#pragma pack(pop)

/**
 * @ingroup datatypes
 * Imu response packet.
 */
#pragma pack(push, 1)
struct ImuResPacket2 {
  std::uint8_t version;
  std::uint8_t header;
  std::uint8_t state;
  std::uint16_t size;
  std::vector<ImuPacket2> packets;
  std::uint8_t checksum;
};
#pragma pack(pop)

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_CHANNEL_DEF_H_
