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
#include "mynteye/api/processor/disparity_normalized_processor.h"

#include <utility>

#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

const char DisparityNormalizedProcessor::NAME[] =
    "DisparityNormalizedProcessor";

DisparityNormalizedProcessor::DisparityNormalizedProcessor(
    std::int32_t proc_period)
    : Processor(std::move(proc_period)) {
  VLOG(2) << __func__ << ": proc_period=" << proc_period;
}

DisparityNormalizedProcessor::~DisparityNormalizedProcessor() {
  VLOG(2) << __func__;
}

std::string DisparityNormalizedProcessor::Name() {
  return NAME;
}

Object *DisparityNormalizedProcessor::OnCreateOutput() {
  return new ObjMat();
}

bool DisparityNormalizedProcessor::OnProcess(
    Object *const in, Object *const out,
    std::shared_ptr<Processor> const parent) {
  MYNTEYE_UNUSED(parent)
  const ObjMat *input = Object::Cast<ObjMat>(in);
  ObjMat *output = Object::Cast<ObjMat>(out);
  cv::normalize(input->value, output->value, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  // cv::normalize maybe return empty ==
  output->id = input->id;
  output->data = input->data;
  return !output->value.empty();
}

MYNTEYE_END_NAMESPACE
