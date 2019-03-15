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
#include "mynteye/api/processor/rectify_processor_ocv.h"

#include <utility>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "mynteye/logger.h"
#include "mynteye/device/device.h"

MYNTEYE_BEGIN_NAMESPACE

const char RectifyProcessorOCV::NAME[] = "RectifyProcessorOCV";

RectifyProcessorOCV::RectifyProcessorOCV(
      std::shared_ptr<IntrinsicsBase> intr_left,
      std::shared_ptr<IntrinsicsBase> intr_right,
      std::shared_ptr<Extrinsics> extr,
      std::int32_t proc_period)
    : Processor(std::move(proc_period)),
      calib_model(CalibrationModel::UNKNOW) {
  VLOG(2) << __func__ << ": proc_period=" << proc_period;
  InitParams(
    *std::dynamic_pointer_cast<IntrinsicsPinhole>(intr_left),
    *std::dynamic_pointer_cast<IntrinsicsPinhole>(intr_right),
    *extr);
}

RectifyProcessorOCV::~RectifyProcessorOCV() {
  VLOG(2) << __func__;
}

std::string RectifyProcessorOCV::Name() {
  return NAME;
}

void RectifyProcessorOCV::ReloadImageParams(
      std::shared_ptr<IntrinsicsBase> intr_left,
      std::shared_ptr<IntrinsicsBase> intr_right,
      std::shared_ptr<Extrinsics> extr) {
  InitParams(
    *std::dynamic_pointer_cast<IntrinsicsPinhole>(intr_left),
    *std::dynamic_pointer_cast<IntrinsicsPinhole>(intr_right),
    *extr);
}

Object *RectifyProcessorOCV::OnCreateOutput() {
  return new ObjMat2();
}

bool RectifyProcessorOCV::OnProcess(
    Object *const in, Object *const out,
    std::shared_ptr<Processor> const parent) {
  MYNTEYE_UNUSED(parent)
  const ObjMat2 *input = Object::Cast<ObjMat2>(in);
  ObjMat2 *output = Object::Cast<ObjMat2>(out);
  cv::remap(input->first, output->first, map11, map12, cv::INTER_LINEAR);
  cv::remap(input->second, output->second, map21, map22, cv::INTER_LINEAR);
  output->first_id = input->first_id;
  output->first_data = input->first_data;
  output->second_id = input->second_id;
  output->second_data = input->second_data;
  return true;
}

void RectifyProcessorOCV::InitParams(
    IntrinsicsPinhole in_left,
    IntrinsicsPinhole in_right,
    Extrinsics ex_right_to_left) {
  calib_model = CalibrationModel::PINHOLE;
  in_left.ResizeIntrinsics();
  in_right.ResizeIntrinsics();
  cv::Size size{in_left.width, in_left.height};

  cv::Mat M1 =
      (cv::Mat_<double>(3, 3) << in_left.fx, 0, in_left.cx, 0, in_left.fy,
       in_left.cy, 0, 0, 1);
  cv::Mat M2 =
      (cv::Mat_<double>(3, 3) << in_right.fx, 0, in_right.cx, 0, in_right.fy,
       in_right.cy, 0, 0, 1);
  cv::Mat D1(1, 5, CV_64F, in_left.coeffs);
  cv::Mat D2(1, 5, CV_64F, in_right.coeffs);
  cv::Mat R =
      (cv::Mat_<double>(3, 3) << ex_right_to_left.rotation[0][0],
       ex_right_to_left.rotation[0][1], ex_right_to_left.rotation[0][2],
       ex_right_to_left.rotation[1][0], ex_right_to_left.rotation[1][1],
       ex_right_to_left.rotation[1][2], ex_right_to_left.rotation[2][0],
       ex_right_to_left.rotation[2][1], ex_right_to_left.rotation[2][2]);
  cv::Mat T(3, 1, CV_64F, ex_right_to_left.translation);

  VLOG(2) << "InitParams size: " << size;
  VLOG(2) << "M1: " << M1;
  VLOG(2) << "M2: " << M2;
  VLOG(2) << "D1: " << D1;
  VLOG(2) << "D2: " << D2;
  VLOG(2) << "R: " << R;
  VLOG(2) << "T: " << T;

  cv::Rect left_roi, right_roi;
  cv::stereoRectify(
      M1, D1, M2, D2, size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY,
      0, size, &left_roi, &right_roi);

  cv::initUndistortRectifyMap(M1, D1, R1, P1, size, CV_16SC2, map11, map12);
  cv::initUndistortRectifyMap(M2, D2, R2, P2, size, CV_16SC2, map21, map22);
}

MYNTEYE_END_NAMESPACE
