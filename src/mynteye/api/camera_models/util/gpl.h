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
#pragma once
#ifndef MYNTEYE_CAMERA_MODELS_GPL_H_
#define MYNTEYE_CAMERA_MODELS_GPL_H_

#include <vector>
#include <algorithm>
#include <cmath>
#include <opencv2/core/core.hpp>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

namespace models {

template <class T>
const T clamp(const T &v, const T &a, const T &b) {
  return std::min(b, std::max(a, v));
}

double hypot3(double x, double y, double z);
float hypot3f(float x, float y, float z);

template <class T>
const T normalizeTheta(const T &theta) {
  T normTheta = theta;

  while (normTheta < -M_PI) {
    normTheta += 2.0 * M_PI;
  }
  while (normTheta > M_PI) {
    normTheta -= 2.0 * M_PI;
  }

  return normTheta;
}

double d2r(double deg);
float d2r(float deg);
double r2d(double rad);
float r2d(float rad);

double sinc(double theta);

template <class T>
const T square(const T &x) {
  return x * x;
}

template <class T>
const T cube(const T &x) {
  return x * x * x;
}

template <class T>
const T random(const T &a, const T &b) {
  return static_cast<double>(rand()) / RAND_MAX * (b - a) + a;  // NOLINT
}

template <class T>
const T randomNormal(const T &sigma) {
  T x1, x2, w;

  do {
    x1 = 2.0 * random(0.0, 1.0) - 1.0;
    x2 = 2.0 * random(0.0, 1.0) - 1.0;
    w = x1 * x1 + x2 * x2;
  } while (w >= 1.0 || w == 0.0);

  w = sqrt((-2.0 * log(w)) / w);

  return x1 * w * sigma;
}

void colorDepthImage(
    cv::Mat &imgDepth, cv::Mat &imgColoredDepth, float minRange,  // NOLINT
    float maxRange);

bool colormap(
    const std::string &name, unsigned char idx, float &r, float &g, float &b);  // NOLINT

std::vector<cv::Point2i> bresLine(int x0, int y0, int x1, int y1);
std::vector<cv::Point2i> bresCircle(int x0, int y0, int r);

void fitCircle(
    const std::vector<cv::Point2d> &points, double &centerX, double &centerY,  // NOLINT
    double &radius);  // NOLINT

std::vector<cv::Point2d> intersectCircles(
    double x1, double y1, double r1, double x2, double y2, double r2);

void LLtoUTM(
    double latitude, double longitude, double &utmNorthing, double &utmEasting,  // NOLINT
    std::string &utmZone);  // NOLINT
void UTMtoLL(
    double utmNorthing, double utmEasting, const std::string &utmZone,  // NOLINT
    double &latitude, double &longitude);  // NOLINT
}

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_CAMERA_MODELS_GPL_H_

