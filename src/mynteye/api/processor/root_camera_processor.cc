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
#include "mynteye/api/processor/root_camera_processor.h"

#include <utility>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "mynteye/logger.h"
MYNTEYE_BEGIN_NAMESPACE
const char RootProcessor::NAME[] = "RootProcessor";

RootProcessor::RootProcessor(std::int32_t proc_period)
    : Processor(std::move(proc_period)) {
  // todo
}
RootProcessor::~RootProcessor() {
  VLOG(2) << __func__;
}

std::string RootProcessor::Name() {
  return NAME;
}

Object *RootProcessor::OnCreateOutput() {
    return new ObjMat2();
}
bool RootProcessor::OnProcess(
    Object *const in, Object *const out,
    std::shared_ptr<Processor> const parent) {
  MYNTEYE_UNUSED(parent)
  // const ObjMat2 *input = Object::Cast<ObjMat2>(in);
  // ObjMat2 *output = Object::Cast<ObjMat2>(out);
  // cv::remap(input->first, output->first, map11, map12, cv::INTER_LINEAR);
  // cv::remap(input->second, output->second, map21, map22, cv::INTER_LINEAR);
  // output->first_id = input->first_id;
  // output->first_data = input->first_data;
  // output->second_id = input->second_id;
  // output->second_data = input->second_data;
  return true;
}
MYNTEYE_END_NAMESPACE
