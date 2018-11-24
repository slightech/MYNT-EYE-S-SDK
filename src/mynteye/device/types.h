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
#ifndef MYNTEYE_DEVICE_TYPES_H_
#define MYNTEYE_DEVICE_TYPES_H_
#pragma once

#include <cstdint>
#include <array>
#include <bitset>
#include <string>
#include <vector>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

#define MYNTEYE_PROPERTY(TYPE, NAME) \
 public:                             \
  void set_##NAME(TYPE NAME) {       \
    NAME##_ = NAME;                  \
  }                                  \
  TYPE NAME() const {                \
    return NAME##_;                  \
  }                                  \
                                     \
 private:                            \
  TYPE NAME##_;

/**
 * Version.
 */
class MYNTEYE_API Version {
 public:
  using size_t = std::size_t;
  using value_t = std::uint8_t;

  Version() = default;
  Version(value_t major, value_t minor) : major_(major), minor_(minor) {}
  explicit Version(const std::string &name)
      : major_(parse_part(name, 0)), minor_(parse_part(name, 1)) {}
  virtual ~Version() {}

  bool operator==(const Version &other) const {
    return major_ == other.major_ && minor_ == other.minor_;
  }
  bool operator<=(const Version &other) const {
    if (major_ < other.major_)
      return true;
    if (major_ > other.major_)
      return false;
    return minor_ <= other.minor_;
  }
  bool operator!=(const Version &other) const {
    return !(*this == other);
  }
  bool operator<(const Version &other) const {
    return !(*this == other) && (*this <= other);
  }
  bool operator>(const Version &other) const {
    return !(*this <= other);
  }
  bool operator>=(const Version &other) const {
    return (*this == other) || (*this > other);
  }
  bool is_between(const Version &from, const Version &until) {
    return (from <= *this) && (*this <= until);
  }

  std::string to_string() const;

  static std::vector<std::string> split(const std::string &s);
  static value_t parse_part(const std::string &name, size_t part);

  MYNTEYE_PROPERTY(value_t, major)
  MYNTEYE_PROPERTY(value_t, minor)
};

/**
 * Hardware version.
 */
class MYNTEYE_API HardwareVersion : public Version {
 public:
  using flag_t = std::bitset<8>;

  HardwareVersion() = default;
  HardwareVersion(value_t major, value_t minor, value_t flag = 0)
      : Version(major, minor), flag_(flag) {}
  explicit HardwareVersion(const std::string &name, value_t flag = 0)
      : Version(parse_part(name, 0), parse_part(name, 1)), flag_(flag) {}

  MYNTEYE_PROPERTY(flag_t, flag)
};

/**
 * Type.
 */
class MYNTEYE_API Type {
 public:
  using size_t = std::size_t;
  using value_t = std::uint16_t;

  Type() = default;
  Type(value_t vendor, value_t product) : vendor_(vendor), product_(product) {}
  explicit Type(const std::string &name)
      : vendor_(parse_part(name, 0, 2)), product_(parse_part(name, 2, 2)) {}
  virtual ~Type() {}

  std::string to_string() const;
  static value_t parse_part(const std::string &name, size_t pos, size_t count);

  MYNTEYE_PROPERTY(value_t, vendor)
  MYNTEYE_PROPERTY(value_t, product)
};

/**
 * @ingroup datatypes
 * Device infomation.
 */
struct MYNTEYE_API DeviceInfo {
  std::string name;
  std::string serial_number;
  Version firmware_version;
  HardwareVersion hardware_version;
  Version spec_version;
  Type lens_type;
  Type imu_type;
  std::uint16_t nominal_baseline;
};

/**
 * @ingroup datatypes
 * Image packet.
 */
#pragma pack(push, 1)
struct ImagePacket {
  std::uint8_t header;
  std::uint8_t size;
  std::uint16_t frame_id;
  std::uint64_t timestamp;
  std::uint16_t exposure_time;
  std::uint8_t checksum;

  ImagePacket() = default;
  explicit ImagePacket(std::uint8_t *data) {
    from_data(data);
  }

  void from_data(std::uint8_t *data) {
    std::uint32_t timestamp_l;
    std::uint32_t timestamp_h;

    header = *data;
    size = *(data + 1);
    frame_id = (*(data + 2) << 8) | *(data + 3);
    timestamp_h = (*(data + 4) << 24) | (*(data + 5) << 16) |
                  (*(data + 6) << 8) | *(data + 7);
    timestamp_l = (*(data + 8) << 24) | (*(data + 9) << 16) |
                  (*(data + 10) << 8) | *(data + 11);
    timestamp = (static_cast<std::uint64_t>(timestamp_h) << 32) | timestamp_l;
    exposure_time = (*(data + 12) << 8) | *(data + 13);
    checksum = *(data + 14);
  }
};
#pragma pack(pop)

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
 * Imu segment.
 */
#pragma pack(push, 1)
struct ImuSegment {
  std::uint32_t serial_number;
  std::uint64_t timestamp;
  std::uint8_t flag;
  std::int16_t temperature;
  std::int16_t accel_or_gyro[3];

  ImuSegment() = default;
  explicit ImuSegment(std::uint8_t *data) {
    from_data(data);
  }

  void from_data(std::uint8_t *data) {
    std::uint32_t timestamp_l;
    std::uint32_t timestamp_h;

    serial_number = (*(data) << 24) | (*(data + 1) << 16) | (*(data + 2) << 8) |
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

/**
 * @ingroup datatypes
 * Imu packet.
 */
#pragma pack(push, 1)
struct ImuPacket {
  std::uint8_t count;
  std::vector<ImuSegment> segments;

  ImuPacket() = default;
  explicit ImuPacket(std::uint8_t seg_count, std::uint8_t *data) {
    count = seg_count;
    from_data(data);
  }
  void from_data(std::uint8_t *data) {
        std::size_t seg_n = sizeof(ImuSegment);  // 21
        for (std::size_t i = 0; i < count; i++) {
          segments.push_back(ImuSegment(data + seg_n * i));
        }
  }
};
#pragma pack(pop)

/**
 * @ingroup datatypes
 * Imu response packet.
 */
#pragma pack(push, 1)
struct ImuResPacket {
  std::uint8_t header;
  std::uint8_t state;
  std::uint16_t size;
  std::vector<ImuPacket> packets;
  std::uint8_t checksum;

  ImuResPacket() = default;
  explicit ImuResPacket(std::uint8_t *data) {
    from_data(data);
  }

  void from_data(std::uint8_t *data) {
    header = *data;
    state = *(data + 1);
    size = (*(data + 2) << 8) | *(data + 3);

    std::size_t seg_n = sizeof(ImuSegment);  // 21
    std::uint8_t seg_count = size / seg_n;
    ImuPacket packet(seg_count, data + 4);
    packets.push_back(packet);
    // packet(2);
    checksum = *(data + 4 + size);
  }
};
#pragma pack(pop)

#undef MYNTEYE_PROPERTY

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_TYPES_H_
