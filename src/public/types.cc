#include "mynteye/types.h"

#include <glog/logging.h>

MYNTEYE_BEGIN_NAMESPACE

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
    CASE(COLOR)
    CASE(DEPTH)
    CASE(POINTS)
    CASE(FISHEYE)
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
    CASE(DESIRED_BRIGHTNESS)
    CASE(IR_CONTROL)
    CASE(HDR_MODE)
    CASE(ZERO_DRIFT_CALIBRATION)
    CASE(ERASE_CHIP)
    default:
      CHECK(is_valid(value));
      return "Option::UNKNOWN";
  }
#undef CASE
}

MYNTEYE_END_NAMESPACE
