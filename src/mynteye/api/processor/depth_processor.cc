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

int DISPARITY_MIN = 0;
int DISPARITY_MAX = 64;

DepthProcessor::DepthProcessor(
    std::shared_ptr<struct CameraROSMsgInfoPair> calib_infos,
    std::shared_ptr<int> min_disp,
    std::shared_ptr<int> max_disp,
    std::int32_t proc_period)
    : Processor(std::move(proc_period)),
    calib_infos_(calib_infos),
    min_disp_(min_disp),
    max_disp_(max_disp) {
  VLOG(2) << __func__;
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
    Object *const in, Object *const out,
    std::shared_ptr<Processor> const parent) {
  MYNTEYE_UNUSED(parent)
  const ObjMat *input = Object::Cast<ObjMat>(in);
  ObjMat *output = Object::Cast<ObjMat>(out);
  int rows = input->value.rows;
  int cols = input->value.cols;
  // std::cout << calib_infos_->T_mul_f << std::endl;
  // 0.0793434

  cv::Mat depth_mat = cv::Mat::zeros(rows, cols, CV_16U);
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      float disparity_value = input->value.at<float>(i, j);
      if (disparity_value < (max_disp_ ? *max_disp_ : DISPARITY_MAX) &&
          disparity_value > (min_disp_ ? *min_disp_ : DISPARITY_MIN)) {
        float depth = calib_infos_->T_mul_f / disparity_value;
        depth_mat.at<ushort>(i, j) = depth;
      }
    }
  }
  output->value = depth_mat;
  output->id = input->id;
  output->data = input->data;
  return true;
}

MYNTEYE_END_NAMESPACE
