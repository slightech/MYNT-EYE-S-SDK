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
#include "mynteye/device/standard2/device_s2.h"

#include "mynteye/logger.h"
#include "mynteye/device/motions.h"
#include "mynteye/device/standard2/channels_adapter_s2.h"
#include "mynteye/device/standard2/streams_adapter_s2.h"

MYNTEYE_BEGIN_NAMESPACE

Standard2Device::Standard2Device(const Model &model,
    std::shared_ptr<uvc::device> device)
  : Device(model, device,
           std::make_shared<Standard2StreamsAdapter>(model),
           std::make_shared<Standard2ChannelsAdapter>(model)) {
  VLOG(2) << __func__;
  CHECK(model == Model::STANDARD2 || model == Model::STANDARD210A ||
        model == Model::STANDARD200B);
}

Standard2Device::~Standard2Device() {
  VLOG(2) << __func__;
}

Capabilities Standard2Device::GetKeyStreamCapability() const {
  return Capabilities::STEREO_COLOR;
}

void Standard2Device::OnStereoStreamUpdate() {
  if (motion_tracking_) {
    auto &&motions = this->motions();
    motions->DoMotionTrack();
  }
}

MYNTEYE_END_NAMESPACE
