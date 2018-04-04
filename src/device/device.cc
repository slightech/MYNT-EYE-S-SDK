#include "device/device.h"

#include <glog/logging.h>

#include "uvc/uvc.h"

MYNTEYE_BEGIN_NAMESPACE

Device::Device() {
  VLOG(2) << __func__;
}

Device::~Device() {
  VLOG(2) << __func__;
}

std::shared_ptr<Device> Device::Create(
    const std::string &name, std::shared_ptr<uvc::device> device) {
  return nullptr;
}

MYNTEYE_END_NAMESPACE
