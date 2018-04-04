#ifndef MYNTEYE_TYPES_H_  // NOLINT
#define MYNTEYE_TYPES_H_
#pragma once

#include <cstdint>

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
  /** Provices infrared stream */
  INFRARED,
  /** Provices second infrared stream */
  INFRARED2,
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
 * @brief Camera control options define general configuration controls.
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

MYNTEYE_ENUM_HELPERS(Model)
MYNTEYE_ENUM_HELPERS(Stream)
MYNTEYE_ENUM_HELPERS(Capabilities)
MYNTEYE_ENUM_HELPERS(Info)
MYNTEYE_ENUM_HELPERS(Option)

#undef MYNTEYE_ENUM_HELPERS

/**
 * @defgroup calibration Intrinsics & Extrinsics
 * @brief Intrinsic and extrinsic properties.
 */

/**
 * @ingroup calibration
 * Video stream intrinsics.
 */
struct MYNTEYE_API Intrinsics {
  /** width of the image in pixels */
  std::uint16_t width;
  /** height of the image in pixels */
  std::uint16_t height;
  /** focal length of the image plane, as a multiple of pixel width */
  double fx;
  /** focal length of the image plane, as a multiple of pixel height */
  double fy;
  /** horizontal coordinate of the principal point of the image */
  double cx;
  /** vertical coordinate of the principal point of the image */
  double cy;
  /** distortion model of the image */
  std::uint8_t model;
  /** distortion coefficients: k1,k2,p1,p2,k3 */
  double coeffs[5];
};

/**
 * @ingroup calibration
 * IMU sensor intrinsics: scale, drift and variances.
 */
struct MYNTEYE_API ImuSensorIntrinsics {
  /**
   * Scale X     cross axis  cross axis
   * cross axis  Scale Y     cross axis
   * cross axis  cross axis  Scale Z
   */
  double scale[3][3];
  /* Zero-drift: X, Y, Z */
  double drift[3];

  /** Noise density variances */
  double noise[3];
  /** Random walk variances */
  double bias[3];
};

/**
 * @ingroup calibration
 * IMU intrinsics, including accelerometer and gyroscope.
 */
struct MYNTEYE_API ImuIntrinsics {
  ImuSensorIntrinsics accel;
  ImuSensorIntrinsics gyro;
};

/**
 * @ingroup calibration
 * Cross-stream extrinsics, represent how the different devices are connected.
 */
struct MYNTEYE_API Extrinsics {
  double rotation[3][3]; /**< rotation matrix */
  double translation[3]; /**< translation vector */
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
  /** Image timestamp in 0.01ms */
  std::uint32_t timestamp;
  /** Image exposure time in 0.01ms */
  std::uint16_t exposure_time;
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
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_TYPES_H_ NOLINT
