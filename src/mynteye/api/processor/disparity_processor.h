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
#ifndef MYNTEYE_API_PROCESSOR_DISPARITY_PROCESSOR_H_
#define MYNTEYE_API_PROCESSOR_DISPARITY_PROCESSOR_H_
#pragma once

#include <string>
#include "mynteye/api/processor.h"

namespace cv {

class StereoSGBM;
class StereoBM;
}  // namespace cv

enum class DisparityProcessorType : std::uint8_t {
  /** bm */
  SGBM = 0,
  /** sgbm */
  BM = 1,
  /** unknow */
  UNKNOW
};


MYNTEYE_BEGIN_NAMESPACE

class DisparityProcessor : public Processor {
 public:
  static const char NAME[];

  explicit DisparityProcessor(DisparityProcessorType type,
      std::int32_t proc_period = 0);
  virtual ~DisparityProcessor();

  std::string Name() override;

 protected:
  Object *OnCreateOutput() override;
  bool OnProcess(
      Object *const in, Object *const out, Processor *const parent) override;

 private:
  cv::Ptr<cv::StereoSGBM> sgbm_matcher;
  cv::Ptr<cv::StereoBM> bm_matcher;
  DisparityProcessorType type_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_API_PROCESSOR_DISPARITY_PROCESSOR_H_
