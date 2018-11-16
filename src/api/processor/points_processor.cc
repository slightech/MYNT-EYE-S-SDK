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
#include "api/processor/points_processor.h"

#include <opencv2/calib3d/calib3d.hpp>

#include <utility>

#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

PointsProcessor::PointsProcessor(cv::Mat Q, std::int32_t proc_period)
    : Processor(std::move(proc_period)), Q_(std::move(Q)) {
  VLOG(2) << __func__ << ": proc_period=" << proc_period;
}

PointsProcessor::~PointsProcessor() {
  VLOG(2) << __func__;
}

std::string PointsProcessor::Name() {
  return NAME;
}

Object *PointsProcessor::OnCreateOutput() {
  return new ObjMat();
}

bool PointsProcessor::OnProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(parent)
  const ObjMat *input = Object::Cast<ObjMat>(in);
  ObjMat *output = Object::Cast<ObjMat>(out);
  cv::reprojectImageTo3D(input->value, output->value, Q_, true);
  return true;
}

MYNTEYE_END_NAMESPACE
