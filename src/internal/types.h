#ifndef MYNTEYE_INTERNAL_TYPES_H_  // NOLINT
#define MYNTEYE_INTERNAL_TYPES_H_
#pragma once

#include <cstdint>

#include <bitset>
#include <string>
#include <vector>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

#define MYNTEYE_FOURCC(a, b, c, d)                  \
  ((std::uint32_t)(a) | ((std::uint32_t)(b) << 8) | \
   ((std::uint32_t)(c) << 16) | ((std::uint32_t)(d) << 24))  // NOLINT

/**
 * @ingroup enumerations
 * @brief Formats define how each stream can be encoded.
 */
enum class Format : std::uint32_t {
  YUYV = MYNTEYE_FOURCC('Y', 'U', 'Y', 'V'),
};

#undef MYNTEYE_FOURCC

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
class Version {
 public:
  using size_t = std::size_t;
  using value_t = std::uint8_t;

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
class HardwareVersion : public Version {
 public:
  using flag_t = std::bitset<8>;

  HardwareVersion(value_t major, value_t minor, value_t flag = 0)
      : Version(major, minor), flag_(flag) {}
  explicit HardwareVersion(const std::string &name, value_t flag = 0)
      : Version(parse_part(name, 0), parse_part(name, 1)), flag_(flag) {}

  MYNTEYE_PROPERTY(flag_t, flag)
};

/**
 * Type.
 */
class Type {
 public:
  using size_t = std::size_t;
  using value_t = std::uint16_t;

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
struct DeviceInfo {
  std::string name;
  std::string serial_number;
  Version firmware_version;
  HardwareVersion hardware_version;
  Version spec_version;
  Type lens_type;
  Type imu_type;
  std::uint16_t nominal_baseline;
};

#undef MYNTEYE_PROPERTY

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_TYPES_H_ NOLINT
