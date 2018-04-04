#include "device/device.h"

#include <glog/logging.h>

#include "internal/strings.h"
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
  if (name == "MYNTEYE") {
  } else if (strings::starts_with(name, "MYNT-EYE-")) {
    // TODO(JohnZhao): Create different device by name, such as MYNT-EYE-S1000
  }
  return nullptr;
}

MYNTEYE_END_NAMESPACE
