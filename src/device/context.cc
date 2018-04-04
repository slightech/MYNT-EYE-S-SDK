#include "device/context.h"

#include <glog/logging.h>

#include "device/device.h"
#include "uvc/uvc.h"

MYNTEYE_BEGIN_NAMESPACE

Context::Context() : context_(uvc::create_context()) {
  VLOG(2) << __func__;

  for (auto &&device : uvc::query_devices(context_)) {
    auto name = uvc::get_name(*device);
    auto vid = uvc::get_vendor_id(*device);
    auto pid = uvc::get_product_id(*device);
    // auto video_name = uvc::get_video_name(*device);
    LOG(INFO) << "UVC device detected, name: " << name << ", vid: 0x"
              << std::hex << vid << ", pid: 0x" << std::hex << pid;
    if (vid == MYNTEYE_VID) {
      auto d = Device::Create(name, device);
      if (d) {
        devices_.push_back(d);
      } else {
        LOG(ERROR) << "Device is not supported by MYNT EYE.";
      }
    }
  }
}

Context::~Context() {
  VLOG(2) << __func__;
}

MYNTEYE_END_NAMESPACE
