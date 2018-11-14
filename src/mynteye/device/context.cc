// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "mynteye/device/context.h"

#include "mynteye/logger.h"

#include "mynteye/device/device.h"
#include "mynteye/uvc/uvc.h"

MYNTEYE_BEGIN_NAMESPACE

Context::Context() : context_(uvc::create_context()) {
  VLOG(2) << __func__;

  for (auto &&device : uvc::query_devices(context_)) {
    auto name = uvc::get_name(*device);
    auto vid = uvc::get_vendor_id(*device);
    auto pid = uvc::get_product_id(*device);
    // auto video_name = uvc::get_video_name(*device);
    VLOG(2) << "UVC device detected, name: " << name << ", vid: 0x" << std::hex
            << vid << ", pid: 0x" << std::hex << pid;
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
