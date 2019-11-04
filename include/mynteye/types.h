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

#include <memory.h>
#include <string>

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
  /** Standard 2 */
  STANDARD2,
  /** Standard 210a */
  STANDARD210A,
  /** Standard 200b */
  STANDARD200B,
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
  /** Provide stereo color stream */
  STEREO_COLOR,
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
  /** Auxiliary chip version */
  AUXILIARY_CHIP_VERSION,
  /** Isp version */
  ISP_VERSION,
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
   * <p>
   *   range: [0,48], default: 24
   * </p>
   */
  GAIN,
  /**
   * Image brightness, valid if manual-exposure
   * <p>
   *   range: [0,240], default: 120
   * </p>
   */
  BRIGHTNESS,
  /**
   * Image contrast, valid if manual-exposure
   * <p>
   *   range: [0,254], default: 116
   * </p>
   */
  CONTRAST,

  /**
   * Image frame rate, must set IMU_FREQUENCY together
   * <p>
   *   values: {10,15,20,25,30,35,40,45,50,55,60}, default: 25
   * </p>
   */
  FRAME_RATE,
  /**
   * IMU frequency, must set FRAME_RATE together
   * <p>
   *   values: {100,200,250,333,500}, default: 200
   * </p>
   */
  IMU_FREQUENCY,

  /**
   * Exposure mode
   * <p>
   *   0: enable auto-exposure<br>
   *   1: disable auto-exposure (manual-exposure)
   * </p>
   */
  EXPOSURE_MODE,
  /**
   * Max gain, valid if auto-exposure
   * <p>
   *   range of standard 1: [0,48], default: 48<br>
   *   range of standard 2: [0,255], default: 8
   * </p>
   */
  MAX_GAIN,
  /**
   * Max exposure time, valid if auto-exposure
   * <p>
   *   range of standard 1: [0,240], default: 240<br>
   *   range of standard 2: [0,655], default: 333
   * </p>
   */
  MAX_EXPOSURE_TIME,
  /**
   * min exposure time, valid if auto-exposure
   * <p>
   *   range: [0,655], default: 0<br>
   * </p>
   */
  MIN_EXPOSURE_TIME,
  /**
   * Desired brightness, valid if auto-exposure
   * <p>
   *   range of standard 1: [0,255], default: 192<br>
   *   range of standard 2: [1,255], default: 122
   * </p>
   */
  DESIRED_BRIGHTNESS,

  /**
   * IR control
   * <p>
   *   range: [0,160], default: 0<br>
   * </p>
   */
  IR_CONTROL,
  /**
   * HDR mode
   * <p>
   *   0: normal<br>
   *   1: WDR
   * </p>
   */
  HDR_MODE,

  /**
   * The range of accelerometer
   * <p>
   *   value of standard 1: {4,8,16,32}, default: 8<br>
   *   value of standard 2: {6,12,24,48}, default: 12
   * </p>
   */
  ACCELEROMETER_RANGE,
  /**
   * The range of gyroscope
   * <p>
   *   value of standard 1: {500,1000,2000,4000}, default: 1000<br>
   *   value of standard 2: {250,500,1000,2000,4000}, default: 1000
   * </p>
   */
  GYROSCOPE_RANGE,
  /**
   * The parameter of accelerometer low pass filter
   * <p>
   *   values: {0,1,2}, default: 2
   * </p>
   */
  ACCELEROMETER_LOW_PASS_FILTER,
  /**
   * The parameter of gyroscope low pass filter
   * <p>
   *   values: {23,64}, default: 64
   * </p>
   */
  GYROSCOPE_LOW_PASS_FILTER,

  /**
   * The setting of IIC address
   * <p>
   *   range: [0,127], default: 0
   * </p>
   */
  IIC_ADDRESS_SETTING,

  /** Zero drift calibration */
  ZERO_DRIFT_CALIBRATION,
  /** Erase chip */
  ERASE_CHIP,

  /** Sync timestamp */
  SYNC_TIMESTAMP,

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
  /** BGR 8:8:8, 24 bits per pixel */
  BGR888 = MYNTEYE_FOURCC('B', 'G', 'R', '3'),
  /** RGB 8:8:8, 24 bits per pixel */
  RGB888 = MYNTEYE_FOURCC('R', 'G', 'B', '3'),
  /** Last guard */
  LAST
};

#undef MYNTEYE_FOURCC

/**
 * @ingroup enumerations
 * @brief Process modes.
 */
enum class ProcessMode : std::int32_t {
  PROC_NONE           = 0,
  PROC_IMU_ASSEMBLY   = 1,
  PROC_IMU_TEMP_DRIFT = 2,
  PROC_IMU_ALL        = PROC_IMU_ASSEMBLY | PROC_IMU_TEMP_DRIFT
};

inline
std::int32_t operator&(const std::int32_t& lhs, const ProcessMode& rhs) {
  return lhs & static_cast<std::int32_t>(rhs);
}
inline
std::int32_t operator&(const ProcessMode& lhs, const std::int32_t& rhs) {
  return static_cast<std::int32_t>(lhs) & rhs;
}
inline
std::int32_t operator&(const ProcessMode& lhs, const ProcessMode& rhs) {
  return static_cast<std::int32_t>(lhs) & static_cast<std::int32_t>(rhs);
}

MYNTEYE_API const char *to_string(const Format &value);

inline std::ostream &operator<<(std::ostream &os, const Format &value) {
  return os << to_string(value);
}

MYNTEYE_API std::size_t bytes_per_pixel(const Format &value);

/**
 * Resolution.
 */
struct MYNTEYE_API Resolution {
  /** Width */
  std::uint16_t width;
  /** Height */
  std::uint16_t height;

  bool operator==(const Resolution &other) const {
    return width == other.width && height == other.height;
  }
  bool operator!=(const Resolution &other) const {
    return !(*this == other);
  }
  bool operator<(const Resolution &other) const {
    return (width * height) < (other.width * other.height);
  }
};

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
  /** Stream frames per second */
  std::uint16_t fps;

  StreamRequest() {}

  StreamRequest(
      std::uint16_t width, std::uint16_t height, Format format,
      std::uint16_t fps)
      : width(width), height(height), format(format), fps(fps) {}

  StreamRequest(const Resolution &res, Format format, std::uint16_t fps)
      : width(res.width), height(res.height), format(format), fps(fps) {}

  Resolution GetResolution() const { return {width, height}; }

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
 * Camera calibration model.
 */
enum class CalibrationModel : std::uint8_t {
  /** Pinhole */
  PINHOLE = 0,
  /** Equidistant: KANNALA_BRANDT */
  KANNALA_BRANDT = 1,
  /** Unknow */
  UNKNOW
};

MYNTEYE_API const char *to_string(const CalibrationModel &model);

inline std::ostream &operator<<(std::ostream &os,
    const CalibrationModel &model) {
  return os << to_string(model);
}

struct MYNTEYE_API IntrinsicsBase {
  IntrinsicsBase() {
    calib_model_ = CalibrationModel::UNKNOW;
  }
  virtual ~IntrinsicsBase() {}
  virtual void ResizeIntrinsics() {}

  /** The calibration model */
  CalibrationModel calib_model() const {
    return calib_model_;
  }
  /** The width of the image in pixels */
  std::uint16_t width;
  /** The height of the image in pixels */
  std::uint16_t height;
  /** Resize scale */
  double resize_scale = 1.0;
 protected:
  CalibrationModel calib_model_;
};

MYNTEYE_API
std::ostream &operator<<(std::ostream &os, const IntrinsicsBase &in);

/**
 * @ingroup calibration
 * Stream intrinsics (Pinhole)
 */
struct MYNTEYE_API IntrinsicsPinhole : public IntrinsicsBase {
  IntrinsicsPinhole() {
    calib_model_ = CalibrationModel::PINHOLE;
  }
  void ResizeIntrinsics() {
    width = static_cast<std::uint16_t>(width * resize_scale);
    height = static_cast<std::uint16_t>(height * resize_scale);
    fx *= resize_scale;
    fy *= resize_scale;
    cx *= resize_scale;
    cy *= resize_scale;
    resize_scale = 1.0;
  }
  /** The focal length of the image plane, as a multiple of pixel width */
  double fx;
  /** The focal length of the image plane, as a multiple of pixel height */
  double fy;
  /** The horizontal coordinate of the principal point of the image */
  double cx;
  /** The vertical coordinate of the principal point of the image */
  double cy;
  /** @deprecated Replaced by calib_model_. The distortion model of the image */
  std::uint8_t model;
  /** The distortion coefficients: k1,k2,p1,p2,k3 */
  double coeffs[5];
};

MYNTEYE_API
std::ostream &operator<<(std::ostream &os, const IntrinsicsPinhole &in);

/**
 * @deprecated Replaced by IntrinsicsPinhole.
 */
using Intrinsics = IntrinsicsPinhole;

/**
 * @ingroup calibration
 * Stream intrinsics (Equidistant: KANNALA_BRANDT)
 */
struct MYNTEYE_API IntrinsicsEquidistant : public IntrinsicsBase {
  IntrinsicsEquidistant() {
    calib_model_ = CalibrationModel::KANNALA_BRANDT;
  }
  /** The distortion coefficients: k2,k3,k4,k5,mu,mv,u0,v0 */
  double coeffs[8];
  void ResizeIntrinsics() {
    width = static_cast<std::uint16_t>(width * resize_scale);
    height = static_cast<std::uint16_t>(height * resize_scale);
    coeffs[4] *= resize_scale;
    coeffs[5] *= resize_scale;
    coeffs[6] *= resize_scale;
    coeffs[7] *= resize_scale;
    resize_scale = 1.0;
  }
};

MYNTEYE_API
std::ostream &operator<<(std::ostream &os, const IntrinsicsEquidistant &in);

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
  /** Assembly error [3][3] */
  double assembly[3][3];
  /** Zero-drift: X, Y, Z */
  double drift[3];
  /** Noise density variances */
  double noise[3];
  /** Random walk variances */
  double bias[3];

  /** Temperature drift
   *  \code
   *    0 - Constant value
   *    1 - Slope
   *  \endcode
   */
  double x[2];
  double y[2];
  double z[2];
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
 * @defgroup disparity params
 */
/**
 * @ingroup disparity
 * Camera disparity computing method type.
 */
enum class DisparityComputingMethod : std::uint8_t {
  /** bm */
  SGBM = 0,
  /** sgbm */
  BM = 1,
  /** unknow */
  UNKNOW
};

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
  /** Image timestamp in 1us */
  std::uint64_t timestamp;
  /** Image exposure time, virtual value in [1, 480] */
  std::uint16_t exposure_time;
  /** Is external time source */
  bool is_ets = false;

  void Reset() {
    frame_id = 0;
    timestamp = 0;
    exposure_time = 0;
    is_ets = false;
  }

  ImgData() {
    Reset();
  }
  ImgData(const ImgData &other) {
    frame_id = other.frame_id;
    timestamp = other.timestamp;
    exposure_time = other.exposure_time;
    is_ets = other.is_ets;
  }
  ImgData &operator=(const ImgData &other) {
    frame_id = other.frame_id;
    timestamp = other.timestamp;
    exposure_time = other.exposure_time;
    is_ets = other.is_ets;
    return *this;
  }
};

/**
 * @ingroup datatypes
 * IMU data.
 */
struct MYNTEYE_API ImuData {
  /** IMU frame id */
  std::uint32_t frame_id;
  /**
   * IMU accel or gyro flag
   * <p>
   *   0: accel and gyro are both valid<br>
   *   1: accel is valid<br>
   *   2: gyro is valid
   * </p>
   */
  std::uint8_t flag;
  /** Is external time source */
  bool is_ets = false;
  /** IMU timestamp in 1us */
  std::uint64_t timestamp;
  /** IMU accelerometer data for 3-axis: X, Y, Z. */
  double accel[3];
  /** IMU gyroscope data for 3-axis: X, Y, Z. */
  double gyro[3];
  /** IMU temperature */
  double temperature;

  void Reset() {
    flag = 0;
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

/**
 * @ingroup datatypes
 * ROS camera info.
 */
struct MYNTEYE_API CameraROSMsgInfo {
  /** height */
  unsigned int height = 0;
  /** width */
  unsigned int width = 0;
  /** calib model */
  std::string distortion_model = "null";
  double  D[5] = {0};
  double  K[9] = {0};
  double  R[9] = {0};
  double  P[12] = {0};
};
MYNTEYE_API
std::ostream &operator<<(std::ostream &os, const CameraROSMsgInfo &info);

struct CameraROSMsgInfoPair {
  inline bool isEmpty() {
    return left.height * left.width * right.height * right.width <= 0;
  }
  struct CameraROSMsgInfo left;
  struct CameraROSMsgInfo right;
  double T_mul_f = -1.f;
  double cx1_minus_cx2 = 0.f;
  double  R[9] = {0};
  double  P[12] = {0};
};
MYNTEYE_API
std::ostream &operator<<(std::ostream &os, const CameraROSMsgInfoPair &info);

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_TYPES_H_
