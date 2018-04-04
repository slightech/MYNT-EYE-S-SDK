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

MYNTEYE_END_NAMESPACE
