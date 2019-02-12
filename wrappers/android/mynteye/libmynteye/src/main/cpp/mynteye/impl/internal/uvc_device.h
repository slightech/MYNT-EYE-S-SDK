#pragma once

#include "usb_info.h"

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

struct device;

MYNTEYE_API std::shared_ptr<device> create_device(UsbInfo info);

}  // namespace uvc

MYNTEYE_END_NAMESPACE
