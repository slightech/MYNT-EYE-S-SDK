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
#include "mynteye/api/processor/depth_processor.h"

#include <utility>

#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

const char DepthProcessor::NAME[] = "DepthProcessor";

DepthProcessor::DepthProcessor(std::int32_t proc_period)
    : Processor(std::move(proc_period)) {
  VLOG(2) << __func__ << ": proc_period=" << proc_period;
}

DepthProcessor::~DepthProcessor() {
  VLOG(2) << __func__;
}

std::string DepthProcessor::Name() {
  return NAME;
}

Object *DepthProcessor::OnCreateOutput() {
  return new ObjMat();
}

bool DepthProcessor::OnProcess(
    Object *const in, Object *const out, Processor *const parent) {
  MYNTEYE_UNUSED(parent)
  const ObjMat *input = Object::Cast<ObjMat>(in);
  ObjMat *output = Object::Cast<ObjMat>(out);
  int rows = input->value.rows;
  int cols = input->value.cols;
  float T = 0.08;
  float f = 0.01;
  cv::Mat depth_mat = cv::Mat::zeros(rows, cols, CV_32F);
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      float disparity_value = input->value.at<float>(i, j);
      float depth = T * f / disparity_value;
      depth_mat.at<float>(i, j) = depth;
    }
  }
  output->value = depth_mat;
  output->id = input->id;
  output->data = input->data;
  return true;
}

MYNTEYE_END_NAMESPACE
