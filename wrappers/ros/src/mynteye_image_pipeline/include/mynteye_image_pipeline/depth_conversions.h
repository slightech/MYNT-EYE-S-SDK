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
#ifndef DEPTH_IMAGE_PROC_DEPTH_CONVERSIONS
#define DEPTH_IMAGE_PROC_DEPTH_CONVERSIONS

#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <mynteye_image_pipeline/depth_traits.h>

#include <limits>

namespace mynteye_image_pipeline {

typedef sensor_msgs::PointCloud2 PointCloud;

// Handles float or uint16 depths
template<typename T>
void convert(
    const sensor_msgs::ImageConstPtr& depth_msg,
    PointCloud::Ptr& cloud_msg,
    float fx,float fy, float cx, float cy,
    double range_max = 0.0) {
  // Use correct principal point from calibration
  float center_x = cx;
  float center_y = cy;

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = DepthTraits<T>::toMeters( T(1) );
  float constant_x = unit_scaling / fx;
  float constant_y = unit_scaling / fy;
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step) {
    for (int u = 0; u < (int)cloud_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z) {
      T depth = depth_row[u];

      // Missing points denoted by NaNs
      if (!DepthTraits<T>::valid(depth)) {
        if (range_max != 0.0) {
          depth = DepthTraits<T>::fromMeters(range_max);
        } else {
          *iter_x = *iter_y = *iter_z = bad_point;
          continue;
        }
      }

      // Fill in XYZ
      *iter_x = (u - center_x) * depth * constant_x;
      *iter_y = (v - center_y) * depth * constant_y;
      *iter_z = DepthTraits<T>::toMeters(depth);
    }
  }
}

}  // namespace mynteye_image_pipeline

#endif  // DEPTH_IMAGE_PROC_DEPTH_CONVERSIONS
