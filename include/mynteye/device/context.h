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
#ifndef MYNTEYE_DEVICE_CONTEXT_H_
#define MYNTEYE_DEVICE_CONTEXT_H_
#pragma once

#include <memory>
#include <vector>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

struct context;

}  // namespace uvc

class Device;

/**
 * The context about devices.
 */
class MYNTEYE_API Context {
 public:
  Context();
  ~Context();

  /**
   * Get all devices now.
   * @return a vector of all devices.
   */
  std::vector<std::shared_ptr<Device>> devices() const {
    return devices_;
  }

 private:
  std::shared_ptr<uvc::context> context_;
  std::vector<std::shared_ptr<Device>> devices_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_CONTEXT_H_
