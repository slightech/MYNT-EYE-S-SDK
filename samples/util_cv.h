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
#ifndef MYNTEYE_TUTORIALS_CV_PAINTER_H_  // NOLINT
#define MYNTEYE_TUTORIALS_CV_PAINTER_H_
#pragma once

#include <string>

#include <opencv2/core/core.hpp>

#include "mynteye/types.h"

class CVPainter {
 public:
  typedef enum Gravity {
    TOP_LEFT,
    TOP_RIGHT,
    BOTTOM_LEFT,
    BOTTOM_RIGHT
  } gravity_t;

  typedef struct Angle {
    double angle_x;
    double angle_y;
    double angle_z;
  } angle_t;

  explicit CVPainter(std::int32_t frame_rate = 0);
  ~CVPainter();

  cv::Rect DrawSize(const cv::Mat &img, const gravity_t &gravity = TOP_LEFT);
  cv::Rect DrawImgData(
      const cv::Mat &img, const mynteye::ImgData &data,
      const gravity_t &gravity = TOP_LEFT);
  cv::Rect DrawImuData(
    const cv::Mat &img, const mynteye::ImuData &data,
    const angle_t &offset, const gravity_t &gravity = TOP_RIGHT);
  cv::Rect DrawImuData(
      const cv::Mat &img,
      const mynteye::ImuData &data, const gravity_t &gravity = TOP_RIGHT);
  cv::Rect DrawText(
      const cv::Mat &img, const std::string &text,
      const gravity_t &gravity = TOP_LEFT, const int &margin = 5,
      const int &offset_x = 0, const int &offset_y = 0);

 private:
  std::int32_t frame_rate_;
};

#endif  // MYNTEYE_TUTORIALS_CV_PAINTER_H_ NOLINT
