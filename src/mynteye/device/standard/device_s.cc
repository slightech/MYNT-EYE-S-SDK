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
#include "mynteye/device/standard/device_s.h"

#include "mynteye/logger.h"
#include "mynteye/device/motions.h"
#include "mynteye/device/standard/channels_adapter_s.h"
#include "mynteye/device/standard/streams_adapter_s.h"

MYNTEYE_BEGIN_NAMESPACE

StandardDevice::StandardDevice(std::shared_ptr<uvc::device> device)
  : Device(Model::STANDARD, device,
           std::make_shared<StandardStreamsAdapter>(),
           std::make_shared<StandardChannelsAdapter>()) {
  VLOG(2) << __func__;
}

StandardDevice::~StandardDevice() {
  VLOG(2) << __func__;
}

Capabilities StandardDevice::GetKeyStreamCapability() const {
  return Capabilities::STEREO;
}

void StandardDevice::OnStereoStreamUpdate() {
  if (motion_tracking_) {
    auto &&motions = this->motions();
    motions->DoMotionTrack();
  }
}

MYNTEYE_END_NAMESPACE
