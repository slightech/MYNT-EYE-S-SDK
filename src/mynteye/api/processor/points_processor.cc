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
  static inline void initializeBuffer(std::vector<uint16_t>& buffer) {} // Do nothing - already zero-filled
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

PointsProcessor::PointsProcessor(
    std::shared_ptr<struct CameraROSMsgInfoPair> calib_infos,
    std::int32_t proc_period)
    : Processor(std::move(proc_period)),
    calib_infos_(calib_infos) {
  VLOG(2) << __func__;
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
  Object *const in, Object *const out,
  std::shared_ptr<Processor> const parent) {
  MYNTEYE_UNUSED(parent)

  float fx = calib_infos_->left.K[0];
  float fy = calib_infos_->left.K[4];
  float cx = calib_infos_->left.K[2];
  float cy = calib_infos_->left.K[5];

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
      float depth = input->value.at<uint16_t>(v, u);

      // Missing points denoted by NaNs
      if (!DepthTraits<uint16_t>::valid(depth)) {
        dptr[u][0] = 0;
        dptr[u][1] = 0;
        dptr[u][2] = 0;
        continue;
      }
      dptr[u][0] = (u - center_x) * depth * constant_x ;
      dptr[u][1] = (v - center_y) * depth * constant_y ;
      dptr[u][2] = depth ;
    }
  }
  output->id = input->id;
  output->data = input->data;
  return true;
}

MYNTEYE_END_NAMESPACE
