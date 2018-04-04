#include "device/device_s.h"

#include <glog/logging.h>

MYNTEYE_BEGIN_NAMESPACE

StandardDevice::StandardDevice(std::shared_ptr<uvc::device> device)
    : Device(Model::STANDARD, device) {
  VLOG(2) << __func__;
}

StandardDevice::~StandardDevice() {
  VLOG(2) << __func__;
}

MYNTEYE_END_NAMESPACE
