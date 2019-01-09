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
#include <vector>
#include <limits>

#include <opencv2/calib3d/calib3d.hpp>

#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

namespace {
// Encapsulate differences between processing float and uint16_t depths
template<typename T> struct DepthTraits {};

template<>
struct DepthTraits<uint16_t> {
  static inline bool valid(uint16_t depth) { return depth != 0; }
  static inline float toMeters(uint16_t depth) { return depth * 0.001f; } // originally mm
  static inline uint16_t fromMeters(float depth) { return (depth * 1000.0f) + 0.5f; }
  static inline void initializeBuffer(std::vector<uint8_t>& buffer) {} // Do nothing - already zero-filled
};

template<>
struct DepthTraits<float> {
  static inline bool valid(float depth) { return std::isfinite(depth); }
  static inline float toMeters(float depth) { return depth; }
  static inline float fromMeters(float depth) { return depth; }

  static inline void initializeBuffer(std::vector<uint8_t>& buffer) {
    float* start = reinterpret_cast<float*>(&buffer[0]);
    float* end = reinterpret_cast<float*>(&buffer[0] + buffer.size());
    std::fill(start, end, std::numeric_limits<float>::quiet_NaN());
  }
};

}; // namespace

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
  MYNTEYE_UNUSED(parent)

  float fx = 3.6797709792391299e+02;
  float fy = 3.6808712539453859e+02;
  float cx = 3.7414963027144353e+02;
  float cy = 2.3125000326472903e+02;

  // Use correct principal point from calibration
  float center_x = cx;
  float center_y = cy;

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = DepthTraits<float>::toMeters(static_cast<float>(1));
  float constant_x = unit_scaling / fx;
  float constant_y = unit_scaling / fy;
  // float bad_point = std::numeric_limits<float>::quiet_NaN();

  const ObjMat *input = Object::Cast<ObjMat>(in);
  ObjMat *output = Object::Cast<ObjMat>(out);
  output->value.create(input->value.size(), CV_MAKETYPE(CV_32F, 3));

  int height = static_cast<int>(output->value.rows);
  int width = static_cast<int>(output->value.cols);
  for (int v = 0; v < height; ++v) {

    cv::Vec3f *dptr = output->value.ptr<cv::Vec3f>(v);
    for (int u = 0; u < width; ++u) {
      float depth = input->value.at<float>(v, u);

      // Missing points denoted by NaNs
      if (!DepthTraits<float>::valid(depth)) {
        continue;
      }

      dptr[u][0] = (u - center_x) * depth * constant_x;
      dptr[u][1] = (v - center_y) * depth * constant_y;
      dptr[u][2] = DepthTraits<float>::toMeters(depth);
    }
  }

  return true;
}

MYNTEYE_END_NAMESPACE
