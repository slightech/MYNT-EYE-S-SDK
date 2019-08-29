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
#include "mynteye/types.h"

#include <iomanip>
#include <limits>

#include "mynteye/logger.h"

#define FULL_PRECISION \
  std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)

MYNTEYE_BEGIN_NAMESPACE

const char *to_string(const Model &value) {
#define CASE(X)  \
  case Model::X: \
    return "Model::" #X;
  switch (value) {
    CASE(STANDARD)
    CASE(STANDARD2)
    CASE(STANDARD210A)
    CASE(STANDARD200B)
    default:
      CHECK(is_valid(value));
      return "Model::UNKNOWN";
  }
#undef CASE
}

const char *to_string(const Stream &value) {
#define CASE(X)   \
  case Stream::X: \
    return "Stream::" #X;
  switch (value) {
    CASE(LEFT)
    CASE(RIGHT)
    CASE(LEFT_RECTIFIED)
    CASE(RIGHT_RECTIFIED)
    CASE(DISPARITY)
    CASE(DISPARITY_NORMALIZED)
    CASE(DEPTH)
    CASE(POINTS)
    default:
      CHECK(is_valid(value));
      return "Stream::UNKNOWN";
  }
#undef CASE
}

const char *to_string(const Capabilities &value) {
#define CASE(X)         \
  case Capabilities::X: \
    return "Capabilities::" #X;
  switch (value) {
    CASE(STEREO)
    CASE(STEREO_COLOR)
    CASE(COLOR)
    CASE(DEPTH)
    CASE(POINTS)
    CASE(FISHEYE)
    CASE(INFRARED)
    CASE(INFRARED2)
    CASE(IMU)
    default:
      CHECK(is_valid(value));
      return "Capabilities::UNKNOWN";
  }
#undef CASE
}

const char *to_string(const Info &value) {
#define CASE(X) \
  case Info::X: \
    return "Info::" #X;
  switch (value) {
    CASE(DEVICE_NAME)
    CASE(SERIAL_NUMBER)
    CASE(FIRMWARE_VERSION)
    CASE(HARDWARE_VERSION)
    CASE(SPEC_VERSION)
    CASE(LENS_TYPE)
    CASE(IMU_TYPE)
    CASE(NOMINAL_BASELINE)
    CASE(AUXILIARY_CHIP_VERSION)
    CASE(ISP_VERSION)
    default:
      CHECK(is_valid(value));
      return "Info::UNKNOWN";
  }
#undef CASE
}

const char *to_string(const Option &value) {
#define CASE(X)   \
  case Option::X: \
    return "Option::" #X;
  switch (value) {
    CASE(GAIN)
    CASE(BRIGHTNESS)
    CASE(CONTRAST)
    CASE(FRAME_RATE)
    CASE(IMU_FREQUENCY)
    CASE(EXPOSURE_MODE)
    CASE(MAX_GAIN)
    CASE(MAX_EXPOSURE_TIME)
    CASE(MIN_EXPOSURE_TIME)
    CASE(DESIRED_BRIGHTNESS)
    CASE(IR_CONTROL)
    CASE(HDR_MODE)
    CASE(ACCELEROMETER_RANGE)
    CASE(GYROSCOPE_RANGE)
    CASE(ACCELEROMETER_LOW_PASS_FILTER)
    CASE(GYROSCOPE_LOW_PASS_FILTER)
    CASE(IIC_ADDRESS_SETTING)
    CASE(ZERO_DRIFT_CALIBRATION)
    CASE(ERASE_CHIP)
    CASE(SYNC_TIMESTAMP)
    default:
      CHECK(is_valid(value));
      return "Option::UNKNOWN";
  }
#undef CASE
}

const char *to_string(const Source &value) {
#define CASE(X)   \
  case Source::X: \
    return "Source::" #X;
  switch (value) {
    CASE(VIDEO_STREAMING)
    CASE(MOTION_TRACKING)
    CASE(ALL)
    default:
      return "Source::UNKNOWN";
  }
#undef CASE
}

const char *to_string(const AddOns &value) {
#define CASE(X)   \
  case AddOns::X: \
    return "AddOns::" #X;
  switch (value) {
    CASE(INFRARED)
    CASE(INFRARED2)
    default:
      return "AddOns::UNKNOWN";
  }
#undef CASE
}

const char *to_string(const Format &value) {
#define CASE(X)   \
  case Format::X: \
    return "Format::" #X;
  switch (value) {
    CASE(GREY)
    CASE(YUYV)
    CASE(BGR888)
    CASE(RGB888)
    default:
      return "Format::UNKNOWN";
  }
#undef CASE
}

std::size_t bytes_per_pixel(const Format &value) {
  switch (value) {
    case Format::GREY:
      return 1;
    case Format::YUYV:
      return 2;
    case Format::BGR888:
      return 3;
    default:
      LOG(FATAL) << "Unknown format";
  }
}

std::ostream &operator<<(std::ostream &os, const StreamRequest &request) {
  return os << "width: " << request.width << ", height: " << request.height
      << ", format: " << request.format << ", fps: " << request.fps;
}

const char *to_string(const CalibrationModel &model) {
#define CASE(X)   \
  case CalibrationModel::X: \
    return "CalibrationModel::" #X;
  switch (model) {
    CASE(PINHOLE)
    CASE(KANNALA_BRANDT)
    default:
      return "CalibrationModel::UNKNOWN";
  }
#undef CASE
}

std::ostream &operator<<(std::ostream &os, const IntrinsicsBase &in) {
  switch (in.calib_model()) {
    case CalibrationModel::PINHOLE:
      return os << dynamic_cast<const IntrinsicsPinhole &>(in);
    case CalibrationModel::KANNALA_BRANDT:
      return os << dynamic_cast<const IntrinsicsEquidistant &>(in);
    default:
      return os << "unknown calib model";
  }
}

std::ostream &operator<<(std::ostream &os, const IntrinsicsPinhole &in) {
  os << "pinhole, " << FULL_PRECISION
     << "width: " << in.width << ", height: " << in.height
     << ", fx: " << in.fx << ", fy: " << in.fy << ", cx: " << in.cx
     << ", cy: " << in.cy << ", model: " << static_cast<int>(in.model)
     << ", coeffs: [";
  for (int i = 0; i <= 3; i++)
    os << in.coeffs[i] << ", ";
  return os << in.coeffs[4] << "]";
}

std::ostream &operator<<(std::ostream &os, const IntrinsicsEquidistant &in) {
  os << "equidistant, " << FULL_PRECISION
     << "width: " << in.width << ", height: " << in.height
     << ", k2: " << in.coeffs[0] << ", k3: " << in.coeffs[1]
     << ", k4: " << in.coeffs[2] << ", k5: " << in.coeffs[3]
     << ", mu: " << in.coeffs[4] << ", mv: " << in.coeffs[5]
     << ", u0: " << in.coeffs[6] << ", v0: " << in.coeffs[7];
  return os;
}

std::ostream &operator<<(std::ostream &os, const ImuIntrinsics &in) {
  os << FULL_PRECISION << "scale: [";
  for (int i = 0; i <= 2; i++)
    os << in.scale[0][i] << ", ";
  for (int i = 0; i <= 2; i++)
    os << in.scale[1][i] << ", ";
  for (int i = 0; i <= 1; i++)
    os << in.scale[2][i] << ", ";
  os << in.scale[2][2] << "]";

  os << ", assembly: [";
  for (int i = 0; i <= 2; i++)
    os << in.assembly[0][i] << ", ";
  for (int i = 0; i <= 2; i++)
    os << in.assembly[1][i] << ", ";
  for (int i = 0; i <= 1; i++)
    os << in.assembly[2][i] << ", ";
  os << in.assembly[2][2] << "]";

  os << ", drift: [";
  for (int i = 0; i <= 1; i++)
    os << in.drift[i] << ", ";
  os << in.drift[2] << "]";

  os << ", noise: [";
  for (int i = 0; i <= 1; i++)
    os << in.noise[i] << ", ";
  os << in.noise[2] << "]";

  os << ", bias: [";
  for (int i = 0; i <= 1; i++)
    os << in.bias[i] << ", ";
  os << in.bias[2] << "]";

  os << ", x: [";
  for (int i = 0; i <= 0; i++)
    os << in.x[i] << ", ";
  os << in.x[1] << "]";

  os << ", y: [";
  for (int i = 0; i <= 0; i++)
    os << in.y[i] << ", ";
  os << in.y[1] << "]";

  os << ", z: [";
  for (int i = 0; i <= 0; i++)
    os << in.z[i] << ", ";
  os << in.z[1] << "]";

  return os;
}

std::ostream &operator<<(std::ostream &os, const MotionIntrinsics &in) {
  return os << FULL_PRECISION << "accel: {" << in.accel << "}, gyro: {"
            << in.gyro << "}";
}

std::ostream &operator<<(std::ostream &os, const Extrinsics &ex) {
  os << FULL_PRECISION << "rotation: [";
  for (int i = 0; i <= 2; i++)
    os << ex.rotation[0][i] << ", ";
  for (int i = 0; i <= 2; i++)
    os << ex.rotation[1][i] << ", ";
  for (int i = 0; i <= 1; i++)
    os << ex.rotation[2][i] << ", ";
  os << ex.rotation[2][2] << "]";

  os << ", translation: [";
  for (int i = 0; i <= 1; i++)
    os << ex.translation[i] << ", ";
  os << ex.translation[2] << "]";

  return os;
}

std::ostream &operator<<(std::ostream &os, const OptionInfo &info) {
  return os << FULL_PRECISION << "min: " << info.min << ", max: " << info.max
            << ", def: " << info.def;
}

std::ostream &operator<<(std::ostream &os, const CameraROSMsgInfo &info) {
  os << FULL_PRECISION << "width: " << info.width << ", height: "
    << info.height << std::endl << "distortion_model: "
    << info.distortion_model;
  os << std::endl << "D: ";
  for (size_t i = 0; i < 5; i++)
    os << info.D[i] << ",";
  os << std::endl << "K: ";
  for (size_t i = 0; i < 9; i++)
    os << info.K[i] << ",";
  os << std::endl << "R: ";
  for (size_t i = 0; i < 9; i++)
    os << info.R[i] << ",";
  os << std::endl << "P: ";
  for (size_t i = 0; i < 12; i++)
    os << info.P[i] << ",";
  os << std::endl;

  return os;
}

std::ostream &operator<<(std::ostream &os, const CameraROSMsgInfoPair &info) {
  os << "left:\n" << info.left << std::endl;
  os << "right:\n" << info.right << std::endl;
  return os;
}

MYNTEYE_END_NAMESPACE
