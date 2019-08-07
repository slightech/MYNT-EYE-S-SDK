#pragma once

#include <memory>

#include "mynteye/logger.h"
#include "mynteye/types.h"

#include "addon.hpp"
#include "calibration_model.hpp"
#include "capability.hpp"
#include "device_usb_info.hpp"
#include "format.hpp"
#include "info.hpp"
#include "model.hpp"
#include "option.hpp"
#include "source.hpp"
#include "stream.hpp"

#include "internal/usb_info.h"

namespace mynteye_jni {

// device_usb_info

using RawUsbInfo = MYNTEYE_NAMESPACE::UsbInfo;
using JniUsbInfo = mynteye_jni::DeviceUsbInfo;

inline
RawUsbInfo from_jni(const JniUsbInfo& info) {
  RawUsbInfo raw;
  raw.vid = info.vid;
  raw.pid = info.pid;
  raw.fd = info.fd;
  raw.busnum = info.bus_num;
  raw.devaddr = info.dev_num;
  raw.usbfs = info.usb_fs;
  raw.name = info.name;
  raw.serial = info.serial;
  return std::move(raw);
}

inline
JniUsbInfo to_jni(const RawUsbInfo& info) {
  return JniUsbInfo{
    info.vid,
    info.pid,
    info.fd,
    info.busnum,
    info.devaddr,
    info.usbfs,
    info.name,
    info.serial,
  };
}

// model

using RawModel = MYNTEYE_NAMESPACE::Model;
using JniModel = mynteye_jni::Model;

inline
RawModel from_jni(const JniModel& model) {
  switch (model) {
    case JniModel::STANDARD: return RawModel::STANDARD;
    case JniModel::STANDARD2: return RawModel::STANDARD2;
    case JniModel::STANDARD210A: return RawModel::STANDARD210A;
    case JniModel::STANDARD200B: return RawModel::STANDARD200B;
    default:
      LOG(FATAL) << "Model is unknown";
  }
}

inline
JniModel to_jni(const RawModel& model) {
  switch (model) {
    case RawModel::STANDARD: return JniModel::STANDARD;
    case RawModel::STANDARD2: return JniModel::STANDARD2;
    case RawModel::STANDARD210A: return JniModel::STANDARD210A;
    case RawModel::STANDARD200B: return JniModel::STANDARD200B;
    default:
      LOG(FATAL) << "Model is unknown";
  }
}

// format

using RawFormat = MYNTEYE_NAMESPACE::Format;
using JniFormat = mynteye_jni::Format;

inline
RawFormat from_jni(const JniFormat& format) {
  switch (format) {
    case JniFormat::GREY: return RawFormat::GREY;
    case JniFormat::YUYV: return RawFormat::YUYV;
    case JniFormat::BGR888: return RawFormat::BGR888;
    case JniFormat::RGB888: return RawFormat::RGB888;
    default:
      LOG(FATAL) << "Format is unknown";
  }
}

inline
JniFormat to_jni(const RawFormat& format) {
  switch (format) {
    case RawFormat::GREY: return JniFormat::GREY;
    case RawFormat::YUYV: return JniFormat::YUYV;
    case RawFormat::BGR888: return JniFormat::BGR888;
    case RawFormat::RGB888: return JniFormat::RGB888;
    default:
      LOG(FATAL) << "Format is unknown";
  }
}

// source

using RawSource = MYNTEYE_NAMESPACE::Source;
using JniSource = mynteye_jni::Source;

inline
RawSource from_jni(const JniSource& source) {
  switch (source) {
    case JniSource::VIDEO_STREAMING: return RawSource::VIDEO_STREAMING;
    case JniSource::MOTION_TRACKING: return RawSource::MOTION_TRACKING;
    case JniSource::ALL: return RawSource::ALL;
    default:
      LOG(FATAL) << "Source is unknown";
  }
}

inline
JniSource to_jni(const RawSource& source) {
  switch (source) {
    case RawSource::VIDEO_STREAMING: return JniSource::VIDEO_STREAMING;
    case RawSource::MOTION_TRACKING: return JniSource::MOTION_TRACKING;
    case RawSource::ALL: return JniSource::ALL;
    default:
      LOG(FATAL) << "Source is unknown";
  }
}

// stream

using RawStream = MYNTEYE_NAMESPACE::Stream;
using JniStream = mynteye_jni::Stream;

inline
RawStream from_jni(const JniStream& stream) {
  switch (stream) {
    case JniStream::LEFT: return RawStream::LEFT;
    case JniStream::RIGHT: return RawStream::RIGHT;
    default:
      LOG(FATAL) << "Stream is unknown";
  }
}

inline
JniStream to_jni(const RawStream& stream) {
  switch (stream) {
    case RawStream::LEFT: return JniStream::LEFT;
    case RawStream::RIGHT: return JniStream::RIGHT;
    default:
      LOG(FATAL) << "Stream is unknown";
  }
}

// capability

using RawCapability = MYNTEYE_NAMESPACE::Capabilities;
using JniCapability = mynteye_jni::Capability;

inline
RawCapability from_jni(const JniCapability& Capability) {
  switch (Capability) {
    case JniCapability::STEREO: return RawCapability::STEREO;
    case JniCapability::STEREO_COLOR: return RawCapability::STEREO_COLOR;
    case JniCapability::COLOR: return RawCapability::COLOR;
    case JniCapability::DEPTH: return RawCapability::DEPTH;
    case JniCapability::POINTS: return RawCapability::POINTS;
    case JniCapability::FISHEYE: return RawCapability::FISHEYE;
    case JniCapability::INFRARED: return RawCapability::INFRARED;
    case JniCapability::INFRARED2: return RawCapability::INFRARED2;
    case JniCapability::IMU: return RawCapability::IMU;
    default:
      LOG(FATAL) << "Capability is unknown";
  }
}

inline
JniCapability to_jni(const RawCapability& Capability) {
  switch (Capability) {
    case RawCapability::STEREO: return JniCapability::STEREO;
    case RawCapability::STEREO_COLOR: return JniCapability::STEREO_COLOR;
    case RawCapability::COLOR: return JniCapability::COLOR;
    case RawCapability::DEPTH: return JniCapability::DEPTH;
    case RawCapability::POINTS: return JniCapability::POINTS;
    case RawCapability::FISHEYE: return JniCapability::FISHEYE;
    case RawCapability::INFRARED: return JniCapability::INFRARED;
    case RawCapability::INFRARED2: return JniCapability::INFRARED2;
    case RawCapability::IMU: return JniCapability::IMU;
    default:
      LOG(FATAL) << "Capability is unknown";
  }
}

// info

using RawInfo = MYNTEYE_NAMESPACE::Info;
using JniInfo = mynteye_jni::Info;

inline
RawInfo from_jni(const JniInfo& Info) {
  switch (Info) {
    case JniInfo::DEVICE_NAME: return RawInfo::DEVICE_NAME;
    case JniInfo::SERIAL_NUMBER: return RawInfo::SERIAL_NUMBER;
    case JniInfo::FIRMWARE_VERSION: return RawInfo::FIRMWARE_VERSION;
    case JniInfo::HARDWARE_VERSION: return RawInfo::HARDWARE_VERSION;
    case JniInfo::SPEC_VERSION: return RawInfo::SPEC_VERSION;
    case JniInfo::LENS_TYPE: return RawInfo::LENS_TYPE;
    case JniInfo::IMU_TYPE: return RawInfo::IMU_TYPE;
    case JniInfo::NOMINAL_BASELINE: return RawInfo::NOMINAL_BASELINE;
    default:
      LOG(FATAL) << "Info is unknown";
  }
}

inline
JniInfo to_jni(const RawInfo& Info) {
  switch (Info) {
    case RawInfo::DEVICE_NAME: return JniInfo::DEVICE_NAME;
    case RawInfo::SERIAL_NUMBER: return JniInfo::SERIAL_NUMBER;
    case RawInfo::FIRMWARE_VERSION: return JniInfo::FIRMWARE_VERSION;
    case RawInfo::HARDWARE_VERSION: return JniInfo::HARDWARE_VERSION;
    case RawInfo::SPEC_VERSION: return JniInfo::SPEC_VERSION;
    case RawInfo::LENS_TYPE: return JniInfo::LENS_TYPE;
    case RawInfo::IMU_TYPE: return JniInfo::IMU_TYPE;
    case RawInfo::NOMINAL_BASELINE: return JniInfo::NOMINAL_BASELINE;
    default:
      LOG(FATAL) << "Info is unknown";
  }
}

// option

using RawOption = MYNTEYE_NAMESPACE::Option;
using JniOption = mynteye_jni::Option;

inline
RawOption from_jni(const JniOption& Option) {
  switch (Option) {
    case JniOption::GAIN:
      return RawOption::GAIN;
    case JniOption::BRIGHTNESS:
      return RawOption::BRIGHTNESS;
    case JniOption::CONTRAST:
      return RawOption::CONTRAST;
    case JniOption::FRAME_RATE:
      return RawOption::FRAME_RATE;
    case JniOption::IMU_FREQUENCY:
      return RawOption::IMU_FREQUENCY;
    case JniOption::EXPOSURE_MODE:
      return RawOption::EXPOSURE_MODE;
    case JniOption::MAX_GAIN:
      return RawOption::MAX_GAIN;
    case JniOption::MAX_EXPOSURE_TIME:
      return RawOption::MAX_EXPOSURE_TIME;
    case JniOption::MIN_EXPOSURE_TIME:
      return RawOption::MIN_EXPOSURE_TIME;
    case JniOption::DESIRED_BRIGHTNESS:
      return RawOption::DESIRED_BRIGHTNESS;
    case JniOption::IR_CONTROL:
      return RawOption::IR_CONTROL;
    case JniOption::HDR_MODE:
      return RawOption::HDR_MODE;
    case JniOption::ACCELEROMETER_RANGE:
      return RawOption::ACCELEROMETER_RANGE;
    case JniOption::GYROSCOPE_RANGE:
      return RawOption::GYROSCOPE_RANGE;
    case JniOption::ACCELEROMETER_LOW_PASS_FILTER:
      return RawOption::ACCELEROMETER_LOW_PASS_FILTER;
    case JniOption::GYROSCOPE_LOW_PASS_FILTER:
      return RawOption::GYROSCOPE_LOW_PASS_FILTER;
    case JniOption::IIC_ADDRESS_SETTING:
      return RawOption::IIC_ADDRESS_SETTING;
    case JniOption::ZERO_DRIFT_CALIBRATION:
      return RawOption::ZERO_DRIFT_CALIBRATION;
    case JniOption::ERASE_CHIP:
      return RawOption::ERASE_CHIP;
    default:
      LOG(FATAL) << "Option is unknown";
  }
}

inline
JniOption to_jni(const RawOption& Option) {
  switch (Option) {
    case RawOption::GAIN:
      return JniOption::GAIN;
    case RawOption::BRIGHTNESS:
      return JniOption::BRIGHTNESS;
    case RawOption::CONTRAST:
      return JniOption::CONTRAST;
    case RawOption::FRAME_RATE:
      return JniOption::FRAME_RATE;
    case RawOption::IMU_FREQUENCY:
      return JniOption::IMU_FREQUENCY;
    case RawOption::EXPOSURE_MODE:
      return JniOption::EXPOSURE_MODE;
    case RawOption::MAX_GAIN:
      return JniOption::MAX_GAIN;
    case RawOption::MAX_EXPOSURE_TIME:
      return JniOption::MAX_EXPOSURE_TIME;
    case RawOption::MIN_EXPOSURE_TIME:
      return JniOption::MIN_EXPOSURE_TIME;
    case RawOption::DESIRED_BRIGHTNESS:
      return JniOption::DESIRED_BRIGHTNESS;
    case RawOption::IR_CONTROL:
      return JniOption::IR_CONTROL;
    case RawOption::HDR_MODE:
      return JniOption::HDR_MODE;
    case RawOption::ACCELEROMETER_RANGE:
      return JniOption::ACCELEROMETER_RANGE;
    case RawOption::GYROSCOPE_RANGE:
      return JniOption::GYROSCOPE_RANGE;
    case RawOption::ACCELEROMETER_LOW_PASS_FILTER:
      return JniOption::ACCELEROMETER_LOW_PASS_FILTER;
    case RawOption::GYROSCOPE_LOW_PASS_FILTER:
      return JniOption::GYROSCOPE_LOW_PASS_FILTER;
    case RawOption::IIC_ADDRESS_SETTING:
      return JniOption::IIC_ADDRESS_SETTING;
    case RawOption::ZERO_DRIFT_CALIBRATION:
      return JniOption::ZERO_DRIFT_CALIBRATION;
    case RawOption::ERASE_CHIP:
      return JniOption::ERASE_CHIP;
    default:
      LOG(FATAL) << "Option is unknown";
  }
}

// addon

using RawAddon = MYNTEYE_NAMESPACE::AddOns;
using JniAddon = mynteye_jni::Addon;

inline
RawAddon from_jni(const JniAddon& Addon) {
  switch (Addon) {
    case JniAddon::INFRARED: return RawAddon::INFRARED;
    case JniAddon::INFRARED2: return RawAddon::INFRARED2;
    default:
      LOG(FATAL) << "Addon is unknown";
  }
}

inline
JniAddon to_jni(const RawAddon& Addon) {
  switch (Addon) {
    case RawAddon::INFRARED: return JniAddon::INFRARED;
    case RawAddon::INFRARED2: return JniAddon::INFRARED2;
    default:
      LOG(FATAL) << "Addon is unknown";
  }
}

// calibration_model

using RawCalibrationModel = MYNTEYE_NAMESPACE::CalibrationModel;
using JniCalibrationModel = mynteye_jni::CalibrationModel;

inline
RawCalibrationModel from_jni(const JniCalibrationModel& CalibrationModel) {
  switch (CalibrationModel) {
    case JniCalibrationModel::PINHOLE:
      return RawCalibrationModel::PINHOLE;
    case JniCalibrationModel::KANNALA_BRANDT:
      return RawCalibrationModel::KANNALA_BRANDT;
    case JniCalibrationModel::UNKNOW:
      return RawCalibrationModel::UNKNOW;
    default:
      LOG(FATAL) << "CalibrationModel is unknown";
  }
}

inline
JniCalibrationModel to_jni(const RawCalibrationModel& CalibrationModel) {
  switch (CalibrationModel) {
    case RawCalibrationModel::PINHOLE:
      return JniCalibrationModel::PINHOLE;
    case RawCalibrationModel::KANNALA_BRANDT:
      return JniCalibrationModel::KANNALA_BRANDT;
    case RawCalibrationModel::UNKNOW:
      return JniCalibrationModel::UNKNOW;
    default:
      LOG(FATAL) << "CalibrationModel is unknown";
  }
}

// others

template<int n>
std::vector<double> to_vector(double (&vector)[n]) {
  std::vector<double> datas;
  for (int i = 0; i < n; i++) {
    datas.push_back(vector[i]);
  }
  return datas;
}

template<int rows, int cols>
std::vector<double> to_vector(double (&matrix)[rows][cols]) {
  std::vector<double> datas;
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      datas.push_back(matrix[i][j]);
    }
  }
  return datas;
}

}  // namespace mynteye_jni
