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
#include <camodocal/camera_models/Camera.h>
#include <camodocal/camera_models/CameraFactory.h>
#include <camodocal/camera_models/CataCamera.h>
#include <camodocal/camera_models/EquidistantCamera.h>
#include <camodocal/camera_models/PinholeCamera.h>
#include <camodocal/gpl/gpl.h>
#include <camodocal/camera_models/Camera.h>
#include <opencv2/core/eigen.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

MYNTEYE_BEGIN_NAMESPACE

struct camera_calib_info {
  unsigned int height = 0;
  unsigned int width = 0;
  std::string distortion_model = "null";
  double  D[4] = {0};
  double  K[9] = {0};
  double  R[9] = {0};
  double  P[12] = {0};
};

struct camera_calib_info_pair {
  struct camera_calib_info left;
  struct camera_calib_info right;
  double T_mul_f;
};

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

  void NotifyImageParamsChanged();

  cv::Mat R1, P1, R2, P2, Q;
  cv::Mat map11, map12, map21, map22;

  inline std::shared_ptr<struct camera_calib_info_pair> getCalibInfoPair() {
    return calib_infos;
  }

 protected:
  Object *OnCreateOutput() override;
  bool OnProcess(
      Object *const in, Object *const out, Processor *const parent) override;

 private:
  void InitParams(IntrinsicsEquidistant in_left,
        IntrinsicsEquidistant in_right, Extrinsics ex_right_to_left);

  cv::Mat rectifyrad(const cv::Mat& R);

  void stereoRectify(camodocal::CameraPtr leftOdo,
      camodocal::CameraPtr rightOdo, const CvMat* K1, const CvMat* K2,
      const CvMat* D1, const CvMat* D2, CvSize imageSize,
      const CvMat* matR, const CvMat* matT,
      CvMat* _R1, CvMat* _R2, CvMat* _P1, CvMat* _P2, double* T_mul_f,
      int flags = cv::CALIB_ZERO_DISPARITY, double alpha = -1,
      CvSize newImgSize = cv::Size());

  Eigen::Matrix4d loadT(const mynteye::Extrinsics& in);
  void loadCameraMatrix(cv::Mat& K, cv::Mat& D,  // NOLINT
      cv::Size& image_size,  // NOLINT
      struct camera_calib_info& calib_data); // NOLINT

  struct camera_calib_info getCalibMatData(
      const mynteye::IntrinsicsEquidistant& in);

  std::shared_ptr<struct camera_calib_info_pair> stereoRectify(
      camodocal::CameraPtr leftOdo,
      camodocal::CameraPtr rightOdo,
      mynteye::IntrinsicsEquidistant in_left,
      mynteye::IntrinsicsEquidistant in_right,
      mynteye::Extrinsics ex_right_to_left);

  camodocal::CameraPtr generateCameraFromIntrinsicsEquidistant(
      const mynteye::IntrinsicsEquidistant & in);

  std::shared_ptr<IntrinsicsBase> intr_left_;
  std::shared_ptr<IntrinsicsBase> intr_right_;
  std::shared_ptr<Extrinsics> extr_;
  CalibrationModel calib_model;
  std::shared_ptr<struct camera_calib_info_pair> calib_infos;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_API_PROCESSOR_RECTIFY_PROCESSOR_H_
