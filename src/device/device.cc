#include "device/device.h"

#include <glog/logging.h>

#include "device/device_s.h"
#include "internal/config.h"
#include "internal/strings.h"
#include "internal/types.h"
#include "uvc/uvc.h"

MYNTEYE_BEGIN_NAMESPACE

Device::Device(const Model &model, std::shared_ptr<uvc::device> device)
    : model_(model), device_(device) {
  VLOG(2) << __func__;
  ReadDeviceInfo();
}

Device::~Device() {
  VLOG(2) << __func__;
}

std::shared_ptr<Device> Device::Create(
    const std::string &name, std::shared_ptr<uvc::device> device) {
  if (name == "MYNTEYE") {
    return std::make_shared<StandardDevice>(device);
  } else if (strings::starts_with(name, "MYNT-EYE-")) {
    // TODO(JohnZhao): Create different device by name, such as MYNT-EYE-S1000
  }
  return nullptr;
}

bool Device::Supports(const Stream &stream) const {
  auto &&supports = stream_supports_map.at(Model::STANDARD);
  return supports.find(stream) != supports.end();
}

bool Device::Supports(const Capabilities &capability) const {
  auto &&supports = capabilities_supports_map.at(Model::STANDARD);
  return supports.find(capability) != supports.end();
}

bool Device::Supports(const Option &option) const {
  auto &&supports = option_supports_map.at(Model::STANDARD);
  return supports.find(option) != supports.end();
}

std::shared_ptr<DeviceInfo> Device::GetInfo() const {
  return device_info_;
}

std::string Device::GetInfo(const Info &info) const {
  CHECK_NOTNULL(device_info_);
  switch (info) {
    case Info::DEVICE_NAME: {
      return device_info_->name;
    } break;
    case Info::SERIAL_NUMBER: {
      return device_info_->serial_number;
    } break;
    case Info::FIRMWARE_VERSION: {
      return device_info_->firmware_version.to_string();
    } break;
    case Info::HARDWARE_VERSION: {
      return device_info_->hardware_version.to_string();
    } break;
    case Info::SPEC_VERSION: {
      return device_info_->spec_version.to_string();
    } break;
    case Info::LENS_TYPE: {
      return device_info_->lens_type.to_string();
    } break;
    case Info::IMU_TYPE: {
      return device_info_->imu_type.to_string();
    } break;
    case Info::NOMINAL_BASELINE: {
      return std::to_string(device_info_->nominal_baseline);
    } break;
    default: { LOG(FATAL) << "Unknown device info"; }
  }
}

ImgIntrinsics Device::GetImgIntrinsics() const {
  return img_intrinsics_;
}

ImgExtrinsics Device::GetImgExtrinsics() const {
  return img_extrinsics_;
}

ImuIntrinsics Device::GetImuIntrinsics() const {
  return imu_intrinsics_;
}

ImuExtrinsics Device::GetImuExtrinsics() const {
  return imu_extrinsics_;
}

void Device::SetStreamCallback(
    const Stream &stream, stream_callback_t callback) {
  if (!Supports(stream)) {
    LOG(WARNING) << "Unsupported stream: " << to_string(stream);
    return;
  }
  if (callback) {
    stream_callbacks_[stream] = callback;
  } else {
    stream_callbacks_.erase(stream);
  }
}

void Device::SetMotionCallback(motion_callback_t callback) {
  motion_callback_ = callback;
}

void Device::Start(const Source &source) {
  if (source == Source::VIDEO_STREAMING) {
    StartVideoStreaming();
  } else if (source == Source::MOTION_TRACKING) {
    StartMotionTracking();
  } else if (source == Source::ALL) {
    Start(Source::VIDEO_STREAMING);
    Start(Source::MOTION_TRACKING);
  } else {
    LOG(FATAL) << "Unsupported source :(";
  }
}

void Device::Stop(const Source &source) {
  if (source == Source::VIDEO_STREAMING) {
    StopVideoStreaming();
  } else if (source == Source::MOTION_TRACKING) {
    StopMotionTracking();
  } else if (source == Source::ALL) {
    Stop(Source::VIDEO_STREAMING);
    Stop(Source::MOTION_TRACKING);
  } else {
    LOG(FATAL) << "Unsupported source :(";
  }
}

StreamRequest Device::GetStreamRequest(const Capabilities &capability) const {
  if (!Supports(capability)) {
    LOG(FATAL) << "Unsupported capability: " << to_string(capability);
  }
  auto &&requests = stream_requests_map.at(Model::STANDARD);
  return requests.at(capability);
}

void Device::StartVideoStreaming() {}

void Device::StopVideoStreaming() {}

void Device::StartMotionTracking() {
  if (!Supports(Capabilities::IMU)) {
    LOG(FATAL) << "IMU is not supported by this device";
  }
  // TODO(JohnZhao)
}

void Device::StopMotionTracking() {
  // TODO(JohnZhao)
}

void Device::ReadDeviceInfo() {
  // TODO(JohnZhao): Read device info
}

void Device::WriteImgIntrinsics(const ImgIntrinsics &intrinsics) {
  // TODO(JohnZhao): Write img intrinsics
  UNUSED(intrinsics);
}

void Device::WriteImgExtrinsics(const ImgExtrinsics &extrinsics) {
  // TODO(JohnZhao): Write img extrinsics
  UNUSED(extrinsics);
}

void Device::WriteImuIntrinsics(const ImuIntrinsics &intrinsics) {
  // TODO(JohnZhao): Write imu intrinsics
  UNUSED(intrinsics);
}

void Device::WriteImuExtrinsics(const ImuExtrinsics &extrinsics) {
  // TODO(JohnZhao): Write imu extrinsics
  UNUSED(extrinsics);
}

MYNTEYE_END_NAMESPACE
