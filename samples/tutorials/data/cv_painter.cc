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
#include "data/cv_painter.h"

#include <glog/logging.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <iomanip>
#include <iostream>
#include <memory>
#include <utility>

#define FONT_FACE cv::FONT_HERSHEY_PLAIN
#define FONT_SCALE 1
#define FONT_COLOR cv::Scalar(255, 255, 255)
#define THICKNESS 1

namespace {

std::shared_ptr<std::ios> NewFormat(int width, int prec, char fillch = ' ') {
  auto fmt = std::make_shared<std::ios>(nullptr);
  fmt->setf(std::ios::fixed);
  fmt->width(std::move(width));
  fmt->precision(std::move(prec));
  fmt->fill(std::move(fillch));
  return fmt;
}

std::ostringstream &Clear(std::ostringstream &os) {
  os.str("");
  os.clear();
  return os;
}

}  // namespace

std::ostream &operator<<(
    std::ostream &os, const std::shared_ptr<std::ios> &fmt) {
  if (fmt)
    os.copyfmt(*fmt);
  return os;
}

CVPainter::CVPainter() {
  VLOG(2) << __func__;
}

CVPainter::~CVPainter() {
  VLOG(2) << __func__;
}

cv::Rect CVPainter::DrawSize(const cv::Mat &img, const gravity_t &gravity) {
  std::ostringstream ss;
  ss << img.cols << "x" << img.rows;
  return DrawText(img, ss.str(), gravity, 5);
}

cv::Rect CVPainter::DrawImgData(
    const cv::Mat &img, const mynteye::ImgData &data,
    const gravity_t &gravity) {
  int sign = 1;
  if (gravity == BOTTOM_LEFT || gravity == BOTTOM_RIGHT)
    sign = -1;

  std::ostringstream ss;
  ss << "frame_id: " << data.frame_id << ", stamp: " << data.timestamp
     << ", expo: " << data.exposure_time;
  cv::Rect rect_i = DrawText(img, ss.str(), gravity, 5);

  Clear(ss) << "size: " << img.cols << "x" << img.rows;
  cv::Rect rect_s =
      DrawText(img, ss.str(), gravity, 5, 0, sign * (5 + rect_i.height));

  // rect_i.width is the max one
  if (sign > 0) {
    return cv::Rect(
        rect_i.tl(),
        cv::Point(rect_i.x + rect_i.width, rect_s.y + rect_s.height));
  } else {
    return cv::Rect(rect_s.tl(), rect_i.br());
  }
}

cv::Rect CVPainter::DrawImuData(
    const cv::Mat &img, const mynteye::ImuData &data,
    const gravity_t &gravity) {
  static std::ostringstream ss;
  static auto fmt_imu = NewFormat(8, 4);
  static auto fmt_temp = NewFormat(6, 4);

  int sign = 1;
  if (gravity == BOTTOM_LEFT || gravity == BOTTOM_RIGHT)
    sign = -1;

  Clear(ss) << "frame_id: " << data.frame_id << ", stamp: " << data.timestamp
            << ", temp: " << fmt_temp << data.temperature;
  cv::Rect rect_i = DrawText(img, ss.str(), gravity, 5);

  Clear(ss) << "accel(x,y,z): " << fmt_imu << data.accel[0] << "," << fmt_imu
            << data.accel[1] << "," << fmt_imu << data.accel[2];
  cv::Rect rect_a =
      DrawText(img, ss.str(), gravity, 5, 0, sign * (5 + rect_i.height));

  Clear(ss) << "gyro(x,y,z): " << fmt_imu << data.gyro[0] << "," << fmt_imu
            << data.gyro[1] << "," << fmt_imu << data.gyro[2];
  cv::Rect rect_g = DrawText(
      img, ss.str(), gravity, 5, 0,
      sign * (10 + rect_i.height + rect_a.height));

  // rect_i.width is the max one
  if (sign > 0) {
    return cv::Rect(
        rect_i.tl(),
        cv::Point(rect_i.x + rect_i.width, rect_g.y + rect_g.height));
  } else {
    return cv::Rect(rect_g.tl(), rect_i.br());
  }
}

cv::Rect CVPainter::DrawText(
    const cv::Mat &img, const std::string &text, const gravity_t &gravity,
    const int &margin, const int &offset_x, const int &offset_y) {
  int w = img.cols, h = img.rows;

  int baseline = 0;
  cv::Size textSize =
      cv::getTextSize(text, FONT_FACE, FONT_SCALE, THICKNESS, &baseline);

  int x, y;
  switch (gravity) {
    case TOP_LEFT:
      x = margin;
      y = margin + textSize.height;
      break;
    case TOP_RIGHT:
      x = w - margin - textSize.width;
      y = margin + textSize.height;
      break;
    case BOTTOM_LEFT:
      x = margin;
      y = h - margin;
      break;
    case BOTTOM_RIGHT:
      x = w - margin - textSize.width;
      y = h - margin;
      break;
    default:  // TOP_LEFT
      x = margin;
      y = margin + textSize.height;
      break;
  }
  x += offset_x;
  y += offset_y;

  cv::Point org(x, y);
#ifdef USE_OPENCV2
  cv::putText(
      const_cast<cv::Mat &>(img), text, org, FONT_FACE, FONT_SCALE, FONT_COLOR,
      THICKNESS);
#else
  cv::putText(img, text, org, FONT_FACE, FONT_SCALE, FONT_COLOR, THICKNESS);
#endif
  return cv::Rect(org, textSize);
}
