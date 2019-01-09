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
#include "mynteye/api/processor/points_processor.h"

#include <utility>

#include <opencv2/calib3d/calib3d.hpp>

#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

const char PointsProcessor::NAME[] = "PointsProcessor";

PointsProcessor::PointsProcessor(std::int32_t proc_period)
    : Processor(std::move(proc_period)) {
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
  return true;
}

MYNTEYE_END_NAMESPACE
