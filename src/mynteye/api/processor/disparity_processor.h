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
#include <memory>
#include <string>
#include <memory>
#include "mynteye/api/processor.h"
#include "mynteye/types.h"

namespace cv {

class StereoSGBM;
class StereoBM;
}  // namespace cv

MYNTEYE_BEGIN_NAMESPACE

class DisparityProcessor : public Processor {
 public:
  static const char NAME[];

  explicit DisparityProcessor(DisparityComputingMethod type,
      std::shared_ptr<struct CameraROSMsgInfoPair> calib_infos,
      std::int32_t proc_period = 0);
  virtual ~DisparityProcessor();

  std::string Name() override;
  void SetDisparityComputingMethodType(
      const DisparityComputingMethod &MethodType);
  void NotifyComputingTypeChanged(const DisparityComputingMethod &MethodType);
  bool ConfigFromFile(const std::string& config);
  std::shared_ptr<int> GetMinDisparity() {
    if (type_ == DisparityComputingMethod::BM) {
      return disparity_min_bm_ptr;
    } else {
      return disparity_min_sgbm_ptr;
    }
  }
  std::shared_ptr<int> GetMaxDisparity() {
    if (type_ == DisparityComputingMethod::BM) {
      return disparity_max_bm_ptr;
    } else {
      return disparity_max_sgbm_ptr;
    }
  }

 protected:
  // inline Processor::process_type ProcessOutputConnection() override {
  //   return Processor::WITHOUT_CLONE;
  // }
  Object *OnCreateOutput() override;
  bool OnProcess(
      Object *const in, Object *const out,
      std::shared_ptr<Processor> const parent) override;

 private:
  cv::Ptr<cv::StereoSGBM> sgbm_matcher;
  cv::Ptr<cv::StereoBM> bm_matcher;
  DisparityComputingMethod type_;
  double cx1_minus_cx2_;
  std::shared_ptr<int> disparity_min_bm_ptr;
  std::shared_ptr<int> disparity_max_bm_ptr;
  std::shared_ptr<int> disparity_min_sgbm_ptr;
  std::shared_ptr<int> disparity_max_sgbm_ptr;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_API_PROCESSOR_DISPARITY_PROCESSOR_H_
