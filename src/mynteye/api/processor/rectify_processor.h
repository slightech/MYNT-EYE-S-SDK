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
#ifndef MYNTEYE_API_PROCESSOR_RECTIFY_PROCESSOR_H_
#define MYNTEYE_API_PROCESSOR_RECTIFY_PROCESSOR_H_
#pragma once

#include <memory>
#include <string>

#include <opencv2/core/core.hpp>

#include "mynteye/types.h"
#include "mynteye/api/processor.h"
#include "mynteye/device/device.h"

#include "mynteye/api/camera_models/equidistant_camera.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

MYNTEYE_BEGIN_NAMESPACE

class Device;

class RectifyProcessor : public Processor {
 public:
  static const char NAME[];

  RectifyProcessor(
      std::shared_ptr<IntrinsicsBase> intr_left,
      std::shared_ptr<IntrinsicsBase> intr_right,
      std::shared_ptr<Extrinsics> extr,
      std::int32_t proc_period = 0);
  virtual ~RectifyProcessor();

  std::string Name() override;

  void ReloadImageParams(
      std::shared_ptr<IntrinsicsBase> intr_left,
      std::shared_ptr<IntrinsicsBase> intr_right,
      std::shared_ptr<Extrinsics> extr);
  void ReloadImageParams() {
    InitParams(in_left_cur, in_right_cur, ex_right_to_left_cur);
  }

  cv::Mat R1, P1, R2, P2, Q;
  cv::Mat map11, map12, map21, map22;
  inline std::shared_ptr<struct CameraROSMsgInfoPair>
      getCameraROSMsgInfoPair() {
    return calib_infos;
  }
  bool SetRectifyAlpha(float alpha);

 protected:
  Object *OnCreateOutput() override;
  bool OnProcess(
      Object *const in, Object *const out,
      std::shared_ptr<Processor> const parent) override;
//   inline Processor::process_type ProcessOutputConnection() override {
//     return Processor::WITHOUT_CLONE;
//   }
//   inline Processor::process_type ProcessInputConnection() override {
//     return Processor::WITHOUT_CLONE;
//   }

 private:
  void InitParams(IntrinsicsEquidistant in_left,
        IntrinsicsEquidistant in_right, Extrinsics ex_right_to_left);

  cv::Mat rectifyrad(const cv::Mat& R);

  void stereoRectify(models::CameraPtr leftOdo,
      models::CameraPtr rightOdo, const CvMat* K1, const CvMat* K2,
      const CvMat* D1, const CvMat* D2, CvSize imageSize,
      const CvMat* matR, const CvMat* matT,
      CvMat* _R1, CvMat* _R2, CvMat* _P1, CvMat* _P2, double* T_mul_f,
      double *cx1_min_cx2,
      int flags = cv::CALIB_ZERO_DISPARITY, double alpha = -1,
      CvSize newImgSize = cvSize(0,0));

// Eigen::Matrix4d loadT(const mynteye::Extrinsics& in);
// subEigen
  models::Matrix4d loadT(const mynteye::Extrinsics &in);

  void loadCameraMatrix(cv::Mat& K, cv::Mat& D,  // NOLINT
      cv::Size& image_size,  // NOLINT
      struct CameraROSMsgInfo& calib_data); // NOLINT

  struct CameraROSMsgInfo getCalibMatData(
      const mynteye::IntrinsicsEquidistant& in);

  std::shared_ptr<struct CameraROSMsgInfoPair> stereoRectify(
      models::CameraPtr leftOdo,
      models::CameraPtr rightOdo,
      mynteye::IntrinsicsEquidistant in_left,
      mynteye::IntrinsicsEquidistant in_right,
      mynteye::Extrinsics ex_right_to_left);

  models::CameraPtr generateCameraFromIntrinsicsEquidistant(
      const mynteye::IntrinsicsEquidistant & in);

  CalibrationModel calib_model;
  std::shared_ptr<struct CameraROSMsgInfoPair> calib_infos;
  double _alpha;
  IntrinsicsEquidistant in_left_cur;
  IntrinsicsEquidistant in_right_cur;
  Extrinsics ex_right_to_left_cur;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_API_PROCESSOR_RECTIFY_PROCESSOR_H_
