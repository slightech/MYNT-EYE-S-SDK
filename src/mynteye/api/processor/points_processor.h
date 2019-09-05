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
#ifndef MYNTEYE_API_PROCESSOR_POINTS_PROCESSOR_H_
#define MYNTEYE_API_PROCESSOR_POINTS_PROCESSOR_H_
#pragma once

#include <string>
#include <memory>
#include <opencv2/core/core.hpp>

#include "mynteye/api/processor.h"
#include "mynteye/types.h"

MYNTEYE_BEGIN_NAMESPACE

class PointsProcessor : public Processor {
 public:
  static const char NAME[];

  explicit PointsProcessor(
      std::shared_ptr<struct CameraROSMsgInfoPair> calib_infos,
      std::int32_t proc_period = 0);
  virtual ~PointsProcessor();

  std::string Name() override;

 protected:
  // inline Processor::process_type ProcessOutputConnection() override {
  //   return Processor::WITHOUT_CLONE;
  // }
  Object *OnCreateOutput() override;
  bool OnProcess(
      Object *const in, Object *const out,
      std::shared_ptr<Processor> const parent) override;

 private:
  std::shared_ptr<struct CameraROSMsgInfoPair> calib_infos_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_API_PROCESSOR_POINTS_PROCESSOR_H_
