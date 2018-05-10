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
#ifndef MYNTEYE_RECTIFY_PROCESSOR_H_  // NOLINT
#define MYNTEYE_RECTIFY_PROCESSOR_H_
#pragma once

#include <opencv2/core/core.hpp>

#include <memory>
#include <string>

#include "api/processor/processor.h"
#include "mynteye/types.h"

MYNTEYE_BEGIN_NAMESPACE

class Device;

class RectifyProcessor : public Processor {
 public:
  static constexpr auto &&NAME = "RectifyProcessor";

  explicit RectifyProcessor(std::shared_ptr<Device> device);
  virtual ~RectifyProcessor();

  std::string Name() override;

  cv::Mat R1, P1, R2, P2, Q;
  cv::Mat map11, map12, map21, map22;

 protected:
  Object *OnCreateOutput() override;
  bool OnProcess(
      Object *const in, Object *const out, Processor *const parent) override;

 private:
  void InitParams(
      Intrinsics in_left, Intrinsics in_right, Extrinsics ex_left_to_right);
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_RECTIFY_PROCESSOR_H_  NOLINT
