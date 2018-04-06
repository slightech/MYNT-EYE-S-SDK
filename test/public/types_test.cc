#include "gtest/gtest.h"

#include "mynteye/types.h"

MYNTEYE_USE_NAMESPACE

TEST(Model, VerifyToString) {
  EXPECT_STREQ("Model::STANDARD", to_string(Model::STANDARD));
}

TEST(Stream, VerifyToString) {
  EXPECT_STREQ("Stream::LEFT", to_string(Stream::LEFT));
  EXPECT_STREQ("Stream::RIGHT", to_string(Stream::RIGHT));
  EXPECT_STREQ("Stream::LEFT_RECTIFIED", to_string(Stream::LEFT_RECTIFIED));
  EXPECT_STREQ("Stream::RIGHT_RECTIFIED", to_string(Stream::RIGHT_RECTIFIED));
  EXPECT_STREQ("Stream::DISPARITY", to_string(Stream::DISPARITY));
  EXPECT_STREQ(
      "Stream::DISPARITY_NORMALIZED", to_string(Stream::DISPARITY_NORMALIZED));
  EXPECT_STREQ("Stream::DEPTH", to_string(Stream::DEPTH));
  EXPECT_STREQ("Stream::POINTS", to_string(Stream::POINTS));
}

TEST(Capabilities, VerifyToString) {
  EXPECT_STREQ("Capabilities::STEREO", to_string(Capabilities::STEREO));
  EXPECT_STREQ("Capabilities::COLOR", to_string(Capabilities::COLOR));
  EXPECT_STREQ("Capabilities::DEPTH", to_string(Capabilities::DEPTH));
  EXPECT_STREQ("Capabilities::POINTS", to_string(Capabilities::POINTS));
  EXPECT_STREQ("Capabilities::FISHEYE", to_string(Capabilities::FISHEYE));
  EXPECT_STREQ("Capabilities::INFRARED", to_string(Capabilities::INFRARED));
  EXPECT_STREQ("Capabilities::INFRARED2", to_string(Capabilities::INFRARED2));
  EXPECT_STREQ("Capabilities::IMU", to_string(Capabilities::IMU));
}

TEST(Info, VerifyToString) {
  EXPECT_STREQ("Info::DEVICE_NAME", to_string(Info::DEVICE_NAME));
  EXPECT_STREQ("Info::SERIAL_NUMBER", to_string(Info::SERIAL_NUMBER));
  EXPECT_STREQ("Info::FIRMWARE_VERSION", to_string(Info::FIRMWARE_VERSION));
  EXPECT_STREQ("Info::HARDWARE_VERSION", to_string(Info::HARDWARE_VERSION));
  EXPECT_STREQ("Info::SPEC_VERSION", to_string(Info::SPEC_VERSION));
  EXPECT_STREQ("Info::LENS_TYPE", to_string(Info::LENS_TYPE));
  EXPECT_STREQ("Info::IMU_TYPE", to_string(Info::IMU_TYPE));
  EXPECT_STREQ("Info::NOMINAL_BASELINE", to_string(Info::NOMINAL_BASELINE));
}

TEST(Option, VerifyToString) {
  EXPECT_STREQ("Option::GAIN", to_string(Option::GAIN));
  EXPECT_STREQ("Option::BRIGHTNESS", to_string(Option::BRIGHTNESS));
  EXPECT_STREQ("Option::CONTRAST", to_string(Option::CONTRAST));
  EXPECT_STREQ("Option::FRAME_RATE", to_string(Option::FRAME_RATE));
  EXPECT_STREQ("Option::IMU_FREQUENCY", to_string(Option::IMU_FREQUENCY));
  EXPECT_STREQ("Option::EXPOSURE_MODE", to_string(Option::EXPOSURE_MODE));
  EXPECT_STREQ("Option::MAX_GAIN", to_string(Option::MAX_GAIN));
  EXPECT_STREQ(
      "Option::MAX_EXPOSURE_TIME", to_string(Option::MAX_EXPOSURE_TIME));
  EXPECT_STREQ(
      "Option::DESIRED_BRIGHTNESS", to_string(Option::DESIRED_BRIGHTNESS));
  EXPECT_STREQ("Option::IR_CONTROL", to_string(Option::IR_CONTROL));
  EXPECT_STREQ("Option::HDR_MODE", to_string(Option::HDR_MODE));
  EXPECT_STREQ(
      "Option::ZERO_DRIFT_CALIBRATION",
      to_string(Option::ZERO_DRIFT_CALIBRATION));
  EXPECT_STREQ("Option::ERASE_CHIP", to_string(Option::ERASE_CHIP));
}

TEST(Format, VerifyToString) {
  EXPECT_STREQ("Format::YUYV", to_string(Format::YUYV));
}
