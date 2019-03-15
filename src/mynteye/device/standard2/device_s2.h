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
#ifndef MYNTEYE_DEVICE_STANDARD2_DEVICE_S2_H_
#define MYNTEYE_DEVICE_STANDARD2_DEVICE_S2_H_
#pragma once

#include <memory>
#include <vector>

#include "mynteye/device/device.h"

MYNTEYE_BEGIN_NAMESPACE

class Standard2Device : public Device {
 public:
  Standard2Device(const Model &model, std::shared_ptr<uvc::device> device);
  virtual ~Standard2Device();

  Capabilities GetKeyStreamCapability() const override;

  void OnStereoStreamUpdate() override;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_STANDARD2_DEVICE_S2_H_
