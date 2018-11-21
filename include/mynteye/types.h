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
#ifndef MYNTEYE_TYPES_H_
#define MYNTEYE_TYPES_H_
#pragma once

#include <cstdint>

#include <algorithm>
#include <iostream>
#include <type_traits>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

/**
 * @defgroup enumerations Enumerations
 * @brief Public enumeration types.
 */

/**
 * @ingroup enumerations
 * @brief Device model.
 */
enum class Model : std::uint8_t {
  /** Standard */
  STANDARD,
  /** Last guard */
  LAST
};

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
  /** Provides stereo stream */
  STEREO,
  /** Provides color stream */
  COLOR,
  /** Provides depth stream */
  DEPTH,
  /** Provides point cloud stream */
  POINTS,
  /** Provides fisheye stream */
  FISHEYE,
  /** Provides infrared stream */
  INFRARED,
  /** Provides second infrared stream */
  INFRARED2,
  /** Provides IMU (accelerometer, gyroscope) data */
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
  LAST
};

/**
 * @ingroup enumerations
 * @brief Camera control options define general configuration controls.
 */
enum class Option : std::uint8_t {
  /**
   * Image gain, valid if manual-exposure
   *
   *   range: [0,48], default: 24
   */
  GAIN,
  /**
   * Image brightness, valid if manual-exposure
   *
   *   range: [0,240], default: 120
   */
  BRIGHTNESS,
  /**
   * Image contrast, valid if manual-exposure
   *
   *   range: [0,255], default: 127
   */
  CONTRAST,
  /**
   * Image frame rate, must set IMU_FREQUENCY together
   *
   *   values: {10,15,20,25,30,35,40,45,50,55,60}, default: 25
   */
  FRAME_RATE,
  /**
   * IMU frequency, must set FRAME_RATE together
   *
   *   values: {100,200,250,333,500}, default: 200
   */
  IMU_FREQUENCY,
  /**
   * Exposure mode
   *
   *   0: enable auto-exposure
   *   1: disable auto-exposure (manual-exposure)
   */
  EXPOSURE_MODE,
  /**
   * Max gain, valid if auto-exposure
   *
   *   range: [0,48], default: 48
   */
  MAX_GAIN,
  /**
   * Max exposure time, valid if auto-exposure
   *
   *   range: [0,240], default: 240
   */
  MAX_EXPOSURE_TIME,
  /**
   * Desired brightness, valid if auto-exposure
   *
   *   range: [0,255], default: 192
   */
  DESIRED_BRIGHTNESS,
  /**
   * IR control
   *
   *   range: [0,160], default: 0
   */
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
  /**
   * The range of accelerometer
   *
   *   values: {4,8,16,32}, default: 8
   */
  ACCELEROMETER_RANGE,
  /**
   * The range of gyroscope
   *
   *   values: {500,1000,2000,4000}, default: 1000
   */
  GYROSCOPE_RANGE,
  /** Last guard */
  LAST
};

/**
 * @ingroup enumerations
 * @brief Source allows the user to choose which data to be captured.
 */
enum class Source : std::uint8_t {
  /** Video streaming of stereo, color, depth, etc. */
  VIDEO_STREAMING,
  /** Motion tracking of IMU (accelerometer, gyroscope) */
  MOTION_TRACKING,
  /** Enable everything together */
  ALL,
  /** Last guard */
  LAST
};

/**
 * @ingroup enumerations
 * @brief Add-Ons are peripheral modules of our hardware.
 */
enum class AddOns : std::uint8_t {
  /** Infrared */
  INFRARED,
  /** Second infrared */
  INFRARED2,
  /** Last guard */
  LAST
};

#define MYNTEYE_ENUM_HELPERS(TYPE)                                       \
  MYNTEYE_API const char *to_string(const TYPE &value);                  \
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

MYNTEYE_ENUM_HELPERS(Model)
MYNTEYE_ENUM_HELPERS(Stream)
MYNTEYE_ENUM_HELPERS(Capabilities)
MYNTEYE_ENUM_HELPERS(Info)
MYNTEYE_ENUM_HELPERS(Option)
MYNTEYE_ENUM_HELPERS(Source)
MYNTEYE_ENUM_HELPERS(AddOns)

#undef MYNTEYE_ENUM_HELPERS

#define MYNTEYE_FOURCC(a, b, c, d)                  \
  ((std::uint32_t)(a) | ((std::uint32_t)(b) << 8) | \
   ((std::uint32_t)(c) << 16) | ((std::uint32_t)(d) << 24))  // NOLINT

/**
 * @ingroup enumerations
 * @brief Formats define how each stream can be encoded.
 */
enum class Format : std::uint32_t {
  /** Greyscale, 8 bits per pixel */
  GREY = MYNTEYE_FOURCC('G', 'R', 'E', 'Y'),
  /** YUV 4:2:2, 16 bits per pixel */
  YUYV = MYNTEYE_FOURCC('Y', 'U', 'Y', 'V'),
  /** Last guard */
  LAST
};

#undef MYNTEYE_FOURCC

MYNTEYE_API const char *to_string(const Format &value);

inline std::ostream &operator<<(std::ostream &os, const Format &value) {
  return os << to_string(value);
}

MYNTEYE_API std::size_t bytes_per_pixel(const Format &value);

/**
 * Stream request.
 */
struct MYNTEYE_API StreamRequest {
  /** Stream width in pixels */
  std::uint16_t width;
  /** Stream height in pixels */
  std::uint16_t height;
  /** Stream pixel format */
  Format format;
  /** Stream frames per second (unused) */
  std::uint16_t fps;

  bool operator==(const StreamRequest &other) const {
    return width == other.width && height == other.height &&
           format == other.format && fps == other.fps;
  }
  bool operator!=(const StreamRequest &other) const {
    return !(*this == other);
  }
};

MYNTEYE_API
std::ostream &operator<<(std::ostream &os, const StreamRequest &request);

/**
 * @defgroup calibration Intrinsics & Extrinsics
 * @brief Intrinsic and extrinsic properties.
 */

/**
 * @ingroup calibration
 * Stream intrinsics,
 */
struct MYNTEYE_API Intrinsics {
  /** The width of the image in pixels */
  std::uint16_t width;
  /** The height of the image in pixels */
  std::uint16_t height;
  /** The focal length of the image plane, as a multiple of pixel width */
  double fx;
  /** The focal length of the image plane, as a multiple of pixel height */
  double fy;
  /** The horizontal coordinate of the principal point of the image */
  double cx;
  /** The vertical coordinate of the principal point of the image */
  double cy;
  /** The distortion model of the image */
  std::uint8_t model;
  /** The distortion coefficients: k1,k2,p1,p2,k3 */
  double coeffs[5];
};

MYNTEYE_API
std::ostream &operator<<(std::ostream &os, const Intrinsics &in);

/**
 * @ingroup calibration
 * IMU intrinsics: scale, drift and variances.
 */
struct MYNTEYE_API ImuIntrinsics {
  /**
   * Scale matrix.
   * \code
   *   Scale X     cross axis  cross axis
   *   cross axis  Scale Y     cross axis
   *   cross axis  cross axis  Scale Z
   * \endcode
   */
  double scale[3][3];
  /* Zero-drift: X, Y, Z */
  double drift[3];

  /** Noise density variances */
  double noise[3];
  /** Random walk variances */
  double bias[3];
};

MYNTEYE_API
std::ostream &operator<<(std::ostream &os, const ImuIntrinsics &in);

/**
 * @ingroup calibration
 * Motion intrinsics, including accelerometer and gyroscope.
 */
struct MYNTEYE_API MotionIntrinsics {
  ImuIntrinsics accel; /**< Accelerometer intrinsics */
  ImuIntrinsics gyro;  /**< Gyroscope intrinsics */
};

MYNTEYE_API
std::ostream &operator<<(std::ostream &os, const MotionIntrinsics &in);

/**
 * @ingroup calibration
 * Extrinsics, represent how the different datas are connected.
 */
struct MYNTEYE_API Extrinsics {
  double rotation[3][3]; /**< Rotation matrix */
  double translation[3]; /**< Translation vector */

  /**
   * Inverse this extrinsics.
   * @return the inversed extrinsics.
   */
  Extrinsics Inverse() const {
    return {{{rotation[0][0], rotation[1][0], rotation[2][0]},
             {rotation[0][1], rotation[1][1], rotation[2][1]},
             {rotation[0][2], rotation[1][2], rotation[2][2]}},
            {-translation[0], -translation[1], -translation[2]}};
  }
};

MYNTEYE_API
std::ostream &operator<<(std::ostream &os, const Extrinsics &ex);

/**
 * @defgroup datatypes Datatypes
 * @brief Public data types.
 */

/**
 * @ingroup datatypes
 * Image data.
 */
struct MYNTEYE_API ImgData {
  /** Image frame id */
  std::uint16_t frame_id;
  /** Image timestamp in 0.01ms */
  std::uint32_t timestamp;
  /** Image exposure time, virtual value in [1, 480] */
  std::uint16_t exposure_time;

  void Reset() {
    frame_id = 0;
    timestamp = 0;
    exposure_time = 0;
  }

  ImgData() {
    Reset();
  }
  ImgData(const ImgData &other) {
    frame_id = other.frame_id;
    timestamp = other.timestamp;
    exposure_time = other.exposure_time;
  }
  ImgData &operator=(const ImgData &other) {
    frame_id = other.frame_id;
    timestamp = other.timestamp;
    exposure_time = other.exposure_time;
    return *this;
  }
};

/**
 * @ingroup datatypes
 * IMU data.
 */
struct MYNTEYE_API ImuData {
  /** Image frame id */
  std::uint16_t frame_id;
  /** IMU timestamp in 0.01ms */
  std::uint32_t timestamp;
  /** IMU accelerometer data for 3-axis: X, Y, Z. */
  double accel[3];
  /** IMU gyroscope data for 3-axis: X, Y, Z. */
  double gyro[3];
  /** IMU temperature */
  double temperature;

  void Reset() {
    frame_id = 0;
    timestamp = 0;
    std::fill(accel, accel + 3, 0);
    std::fill(gyro, gyro + 3, 0);
    temperature = 0;
  }

  ImuData() {
    Reset();
  }
};

/**
 * @ingroup datatypes
 * Option info.
 */
struct MYNTEYE_API OptionInfo {
  /** Minimum value */
  std::int32_t min;
  /** Maximum value */
  std::int32_t max;
  /** Default value */
  std::int32_t def;
};

MYNTEYE_API
std::ostream &operator<<(std::ostream &os, const OptionInfo &info);

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_TYPES_H_
