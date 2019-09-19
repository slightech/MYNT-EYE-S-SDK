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
  calib_infos = std::make_shared<struct CameraROSMsgInfoPair>();
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

struct CameraROSMsgInfo RectifyProcessorOCV::getCalibMatData(
    const mynteye::IntrinsicsPinhole& in) {
  struct CameraROSMsgInfo calib_mat_data;
  calib_mat_data.distortion_model = "PINHOLE";
  calib_mat_data.height = in.height;
  calib_mat_data.width = in.width;

  for (std::size_t i = 0; i < 5; i++) {
    calib_mat_data.D[i] = in.coeffs[i];
  }

  calib_mat_data.K[0] = in.fx;
  calib_mat_data.K[4] = in.cx;
  calib_mat_data.K[2] = in.fy;
  calib_mat_data.K[5] = in.cy;
  calib_mat_data.K[8] = 1;
  return calib_mat_data;
}

void RectifyProcessorOCV::InitParams(
    IntrinsicsPinhole in_left,
    IntrinsicsPinhole in_right,
    Extrinsics ex_right_to_left) {
  calib_model = CalibrationModel::PINHOLE;
  in_left.ResizeIntrinsics();
  in_right.ResizeIntrinsics();
  cv::Size size{in_left.width, in_left.height};

  struct CameraROSMsgInfoPair info_pair;
  info_pair.left = getCalibMatData(in_left);
  info_pair.right = getCalibMatData(in_right);
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

  info_pair.R[0] = ex_right_to_left.rotation[0][0];
  info_pair.R[1] = ex_right_to_left.rotation[0][1];
  info_pair.R[2] = ex_right_to_left.rotation[0][2];
  info_pair.R[3] = ex_right_to_left.rotation[1][0];
  info_pair.R[4] = ex_right_to_left.rotation[1][1];
  info_pair.R[5] = ex_right_to_left.rotation[1][2];
  info_pair.R[6] = ex_right_to_left.rotation[2][0];
  info_pair.R[7] = ex_right_to_left.rotation[2][1];
  info_pair.R[8] = ex_right_to_left.rotation[2][2];

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

  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 4; j++) {
      info_pair.left.P[i*4 + j] = P1.at<double>(i, j);
      info_pair.right.P[i*4 + j] = P2.at<double>(i, j);
    }
  }

  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      info_pair.left.R[i*3 + j] = R1.at<double>(i, j);
      info_pair.right.R[i*3 +j] = R2.at<double>(i, j);
    }
  }

  for (std::size_t i = 0; i< 3 * 4; i++) {
    info_pair.P[i] = info_pair.left.P[i];
  }

  info_pair.T_mul_f = -1.f * in_left.fx * ex_right_to_left.translation[0];
  info_pair.cx1_minus_cx2 = 0.-(in_left.cx - in_right.cx);
  // std::cout << "info_pair.T_mul_f :" << info_pair.T_mul_f << std::endl;
  // std::cout << "info_pair.cx1_minus_cx2 :" << info_pair.cx1_minus_cx2 << std::endl;
  *calib_infos = info_pair;

  cv::initUndistortRectifyMap(M1, D1, R1, P1, size, CV_16SC2, map11, map12);
  cv::initUndistortRectifyMap(M2, D2, R2, P2, size, CV_16SC2, map21, map22);
}

MYNTEYE_END_NAMESPACE
