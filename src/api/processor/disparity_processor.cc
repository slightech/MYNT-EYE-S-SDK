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
#include "api/processor/disparity_processor.h"

#include <opencv2/calib3d/calib3d.hpp>

#include <glog/logging.h>

#include <utility>

MYNTEYE_BEGIN_NAMESPACE

const char DisparityProcessor::NAME[] = "DisparityProcessor";

DisparityProcessor::DisparityProcessor(std::int32_t proc_period)
    : Processor(std::move(proc_period)) {
  VLOG(2) << __func__ << ": proc_period=" << proc_period;

	int blockSize_ = 15;           // 15
	int minDisparity_ = 0;         // 0
	int numDisparities_ = 64;      // 64
	int preFilterSize_ = 9;        // 9
	int preFilterCap_ = 31;        // 31
	int uniquenessRatio_ = 15;     // 15
	int textureThreshold_ = 10;    // 10
	int speckleWindowSize_ = 100;  // 100
	int speckleRange_ = 4;         // 4

	bm_ = cv::StereoBM::create();
	bm_->setBlockSize(blockSize_);
	bm_->setMinDisparity(minDisparity_);
	bm_->setNumDisparities(numDisparities_);
	bm_->setPreFilterSize(preFilterSize_);
	bm_->setPreFilterCap(preFilterCap_);
	bm_->setUniquenessRatio(uniquenessRatio_);
	bm_->setTextureThreshold(textureThreshold_);
	bm_->setSpeckleWindowSize(speckleWindowSize_);
	bm_->setSpeckleRange(speckleRange_);
}

DisparityProcessor::~DisparityProcessor() {
  VLOG(2) << __func__;
}

std::string DisparityProcessor::Name() {
  return NAME;
}

Object *DisparityProcessor::OnCreateOutput() {
  return new ObjMat();
}

bool DisparityProcessor::OnProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(parent)
  const ObjMat2 *input = Object::Cast<ObjMat2>(in);
  ObjMat *output = Object::Cast<ObjMat>(out);

  cv::Mat disparity;
  bm_->compute(input->first, input->second, disparity);
	disparity.convertTo(output->value, CV_32F, 1./16);
  return true;
}

MYNTEYE_END_NAMESPACE
