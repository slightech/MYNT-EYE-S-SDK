#ifndef MYNTEYE_TYPES_H_  // NOLINT
#define MYNTEYE_TYPES_H_
#pragma once

#include <cstdint>

#include <iostream>
#include <type_traits>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

/**
 * @ingroup enumerations
 * @brief Streams define different type of data.
 */
enum class Stream : std::uint8_t {
  /** Left stream */
  LEFT,
  /** Right stream */
  RIGHT,
  /** Left stream, rectified */
  LEFT_RECTIFIED,
  /** Right stream, rectified */
  RIGHT_RECTIFIED,
  /** Disparity stream */
  DISPARITY,
  /** Disparity stream, normalized */
  DISPARITY_NORMALIZED,
  /** Depth stream */
  DEPTH,
  /** Point cloud stream */
  POINTS,
  /** Last guard */
  LAST
};

/**
 * @ingroup enumerations
 * @brief Capabilities define the full set of functionality that the device
 * might provide.
 */
enum class Capabilities : std::uint8_t {
  /** Provices stereo stream */
  STEREO,
  /** Provices color stream */
  COLOR,
  /** Provices depth stream */
  DEPTH,
  /** Provices point cloud stream */
  POINTS,
  /** Provices fisheye stream */
  FISHEYE,
  /** Provices IMU (accelerometer, gyroscope) data */
  IMU,
  /** Last guard */
  LAST
};

/**
 * @ingroup enumerations
 * @brief Camera info fields are read-only strings that can be queried from the
 * device.
 */
enum class Info : std::uint8_t {
  /** Device name */
  DEVICE_NAME,
  /** Serial number */
  SERIAL_NUMBER,
  /** Firmware version */
  FIRMWARE_VERSION,
  /** Hardware version */
  HARDWARE_VERSION,
  /** Spec version */
  SPEC_VERSION,
  /** Lens type */
  LENS_TYPE,
  /** IMU type */
  IMU_TYPE,
  /** Nominal baseline */
  NOMINAL_BASELINE,
  /** Last guard */
  LAST,
};

/**
 * @ingroup enumerations
 * Camera control options define general configuration controls.
 */
enum class Option : std::uint8_t {
  /** Image gain, setting it if manual-exposure */
  GAIN,
  /** Image brightness, setting it if manual-exposure */
  BRIGHTNESS,
  /** Image contrast */
  CONTRAST,
  /** Image frame rate */
  FRAME_RATE,
  /** IMU frequency */
  IMU_FREQUENCY,
  /**
   * Exposure mode
   *
   *   0: enable auto-exposure
   *   1: disable auto-exposure (manual-exposure)
   */
  EXPOSURE_MODE,
  /** Max gain, setting it if auto-exposure */
  MAX_GAIN,
  /** Max exposure time, setting it if auto-exposure */
  MAX_EXPOSURE_TIME,
  /** Desired brightness */
  DESIRED_BRIGHTNESS,
  /** IR control */
  IR_CONTROL,
  /**
   * HDR mode
   *
   *   0: 10-bit
   *   1: 12-bit
   */
  HDR_MODE,
  /** Zero drift calibration */
  ZERO_DRIFT_CALIBRATION,
  /** Erase chip */
  ERASE_CHIP,
  /** Last guard */
  LAST,
};

#define MYNTEYE_ENUM_HELPERS(TYPE)                                       \
  const char *to_string(const TYPE &value);                              \
  inline bool is_valid(const TYPE &value) {                              \
    using utype = std::underlying_type<TYPE>::type;                      \
    utype val = static_cast<utype>(value);                               \
    utype max = static_cast<utype>(TYPE::LAST);                          \
    return /*val >= 0 &&*/ val < max;                                    \
  }                                                                      \
  inline std::ostream &operator<<(std::ostream &os, const TYPE &value) { \
    using utype = std::underlying_type<TYPE>::type;                      \
    if (is_valid(value))                                                 \
      return os << to_string(value);                                     \
    else                                                                 \
      return os << static_cast<utype>(value);                            \
  }

MYNTEYE_ENUM_HELPERS(Stream)
MYNTEYE_ENUM_HELPERS(Capabilities)
MYNTEYE_ENUM_HELPERS(Info)
MYNTEYE_ENUM_HELPERS(Option)

#undef MYNTEYE_ENUM_HELPERS

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_TYPES_H_ NOLINT
