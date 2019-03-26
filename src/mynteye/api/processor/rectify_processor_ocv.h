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
#ifndef MYNTEYE_API_PROCESSOR_RECTIFY_PROCESSOR_OCV_H_
#define MYNTEYE_API_PROCESSOR_RECTIFY_PROCESSOR_OCV_H_
#pragma once

#include <memory>
#include <string>

#include <opencv2/core/core.hpp>

#include "mynteye/types.h"
#include "mynteye/api/processor.h"

MYNTEYE_BEGIN_NAMESPACE

class Device;

class RectifyProcessorOCV : public Processor {
 public:
  static const char NAME[];

  RectifyProcessorOCV(
      std::shared_ptr<IntrinsicsBase> intr_left,
      std::shared_ptr<IntrinsicsBase> intr_right,
      std::shared_ptr<Extrinsics> extr,
      std::int32_t proc_period = 0);
  virtual ~RectifyProcessorOCV();

  std::string Name() override;

  inline std::shared_ptr<struct CameraROSMsgInfoPair>
      getCameraROSMsgInfoPair() {
    return calib_infos;
  }

  struct CameraROSMsgInfo getCalibMatData(
    const mynteye::IntrinsicsPinhole& in);

  void ReloadImageParams(
      std::shared_ptr<IntrinsicsBase> intr_left,
      std::shared_ptr<IntrinsicsBase> intr_right,
      std::shared_ptr<Extrinsics> extr);

  cv::Mat R1, P1, R2, P2, Q;
  cv::Mat map11, map12, map21, map22;

 protected:
  // inline Processor::process_type ProcessOutputConnection() override {
  //   return Processor::WITHOUT_CLONE;
  // }
  // inline Processor::process_type ProcessInputConnection() override {
  //   return Processor::WITHOUT_CLONE;
  // }

  Object *OnCreateOutput() override;
  bool OnProcess(
      Object *const in, Object *const out,
      std::shared_ptr<Processor> const parent) override;

 private:
  void InitParams(IntrinsicsPinhole in_left,
        IntrinsicsPinhole in_right, Extrinsics ex_right_to_left);

  CalibrationModel calib_model;

  std::shared_ptr<struct CameraROSMsgInfoPair> calib_infos;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_API_PROCESSOR_RECTIFY_PROCESSOR_OCV_H_
