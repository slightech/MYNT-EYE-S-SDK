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
#include "util_cv.h"

#include <math.h>

#include <iomanip>
#include <iostream>
#include <memory>
#include <utility>

#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye/logger.h"
#include "mynteye/device/utils.h"

#define FONT_FACE cv::FONT_HERSHEY_PLAIN
#define FONT_SCALE 1
#define FONT_COLOR cv::Scalar(255, 255, 255)
#define THICKNESS 1

#define PI 3.1415926

namespace {

std::shared_ptr<std::ios> NewFormat(int width, int prec, char fillch = ' ') {
  auto fmt = std::make_shared<std::ios>(nullptr);
  fmt->setf(std::ios::fixed);
  if (width > 0)
    fmt->width(std::move(width));
  if (prec > 0)
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

CVPainter::CVPainter(std::int32_t frame_rate)
    : frame_rate_(std::move(frame_rate)) {
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

  static auto fmt_time = NewFormat(0, 2);

  std::ostringstream ss;
  ss << "frame_id: " << data.frame_id;
  ss << ", stamp: " << fmt_time << (0.001f * data.timestamp);  // ms
  ss << ", expo: ";
  if (frame_rate_ == 0) {
    ss << data.exposure_time;
  } else {
    ss << fmt_time << mynteye::utils::get_real_exposure_time(
                          frame_rate_, data.exposure_time);
  }
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
    const angle_t &offset, const gravity_t &gravity) {
  static std::ostringstream ss;
  static auto fmt_imu = NewFormat(8, 4);
  static auto fmt_temp = NewFormat(6, 4);

  int sign = 1;
  if (gravity == BOTTOM_LEFT || gravity == BOTTOM_RIGHT)
    sign = -1;
  static auto fmt_time = NewFormat(0, 2);
  Clear(ss) << "stamp: " << fmt_time << (0.001f * data.timestamp)
            << ", temp: " << fmt_temp << data.temperature;
  cv::Rect rect_i = DrawText(img, ss.str(), gravity, 5);

  static double accel1_s = 0.0;
  static double accel2_s = 0.0;
  static double accel3_s = 0.0;
  static double vector_sum = 0.0;
  static double pitch = 0.0;
  static double roll = 0.0;
  if (data.accel[0] > 0.000001 ||
      data.accel[1] > 0.000001 ||
      data.accel[2] > 0.000001 ||
      data.accel[0] < -0.000001 ||
      data.accel[1] < -0.000001 ||
      data.accel[2] < -0.000001) {
    double accel1_s_square, accel2_s_square, accel3_s_square;

    accel1_s = data.accel[0];
    accel2_s = data.accel[1];
    accel3_s = data.accel[2];
    accel1_s_square = accel1_s * accel1_s;
    accel2_s_square = accel2_s * accel2_s;
    accel3_s_square = accel3_s * accel3_s;
    vector_sum = sqrt(accel1_s_square + accel2_s_square + accel3_s_square);
    pitch = atan(accel1_s / sqrt(accel2_s_square + accel3_s_square))
              * 180.0 / PI;
    roll = atan(accel2_s / sqrt(accel1_s_square + accel3_s_square))
              * 180.0 / PI;
  }

  Clear(ss) << "accel(x,y,z,norm): "
            << fmt_imu << accel1_s << ","
            << fmt_imu << accel2_s << ","
            << fmt_imu << accel3_s << ","
            << fmt_imu << vector_sum;
  cv::Rect rect_a =
      DrawText(img, ss.str(), gravity, 5, 0, sign * (5 + rect_i.height));

  Clear(ss) << "posture(pitch,roll): "
            << fmt_imu << pitch << ","
            << fmt_imu << roll;
  cv::Rect rect_p = DrawText(
      img, ss.str(), gravity, 5, 0,
      sign * (10 + rect_i.height + rect_a.height));

  Clear(ss) << "gyro(x,y,z): " << fmt_imu << data.gyro[0] << "," << fmt_imu
            << data.gyro[1] << "," << fmt_imu << data.gyro[2];
  cv::Rect rect_g = DrawText(
      img, ss.str(), gravity, 5, 0,
      sign * (15 + rect_i.height + rect_a.height + rect_p.height));
  if (offset.angle_x != 0 ||
      offset.angle_y != 0 ||
      offset.angle_z != 0) {
    Clear(ss) << "angle offset(x,y,z): "
              << fmt_imu << offset.angle_x << ","
              << fmt_imu << offset.angle_y << ","
              << fmt_imu << offset.angle_z;
  } else {
    Clear(ss) << "angle offset(x,y,z): "
          << fmt_imu << "x,"
          << fmt_imu << "x,"
          << fmt_imu << "x";
  }

  cv::Rect rect_o = DrawText(
      img, ss.str(), gravity, 5, 0,
      sign * (20 + rect_i.height + rect_a.height +
              rect_p.height + rect_g.height));

  // rect_i.width is the max one
  if (sign > 0) {
    return cv::Rect(
        rect_i.tl(),
        cv::Point(rect_i.x + rect_i.width, rect_o.y + rect_o.height));
  } else {
    return cv::Rect(rect_g.tl(), rect_i.br());
  }
}

cv::Rect CVPainter::DrawImuData(
    const cv::Mat &img, const mynteye::ImuData &data,
    const gravity_t &gravity) {
  angle_t offset = {0, 0, 0};
  return DrawImuData(img, data, offset, gravity);
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
#ifdef WITH_OPENCV2
  cv::putText(
      const_cast<cv::Mat &>(img), text, org, FONT_FACE, FONT_SCALE, FONT_COLOR,
      THICKNESS);
#else
  cv::putText(img, text, org, FONT_FACE, FONT_SCALE, FONT_COLOR, THICKNESS);
#endif
  return cv::Rect(org, textSize);
}
