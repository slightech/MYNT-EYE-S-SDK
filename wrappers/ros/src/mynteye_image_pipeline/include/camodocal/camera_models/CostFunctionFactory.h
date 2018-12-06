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
#ifndef COSTFUNCTIONFACTORY_H
#define COSTFUNCTIONFACTORY_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

#include "camodocal/camera_models/Camera.h"

namespace ceres {
class CostFunction;
}  // namespace ceres

namespace camodocal {

enum {
  CAMERA_INTRINSICS = 1 << 0,
  CAMERA_POSE = 1 << 1,
  POINT_3D = 1 << 2,
  ODOMETRY_INTRINSICS = 1 << 3,
  ODOMETRY_3D_POSE = 1 << 4,
  ODOMETRY_6D_POSE = 1 << 5,
  CAMERA_ODOMETRY_TRANSFORM = 1 << 6
};

class CostFunctionFactory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CostFunctionFactory();

  static boost::shared_ptr<CostFunctionFactory> instance(void);

  ceres::CostFunction *generateCostFunction(
      const CameraConstPtr &camera, const Eigen::Vector3d &observed_P,
      const Eigen::Vector2d &observed_p, int flags) const;

  ceres::CostFunction *generateCostFunction(
      const CameraConstPtr &camera, const Eigen::Vector3d &observed_P,
      const Eigen::Vector2d &observed_p,
      const Eigen::Matrix2d &sqrtPrecisionMat, int flags) const;

  ceres::CostFunction *generateCostFunction(
      const CameraConstPtr &camera, const Eigen::Vector2d &observed_p,
      int flags, bool optimize_cam_odo_z = true) const;

  ceres::CostFunction *generateCostFunction(
      const CameraConstPtr &camera, const Eigen::Vector2d &observed_p,
      const Eigen::Matrix2d &sqrtPrecisionMat, int flags,
      bool optimize_cam_odo_z = true) const;

  ceres::CostFunction *generateCostFunction(
      const CameraConstPtr &camera, const Eigen::Vector3d &odo_pos,
      const Eigen::Vector3d &odo_att, const Eigen::Vector2d &observed_p,
      int flags, bool optimize_cam_odo_z = true) const;

  ceres::CostFunction *generateCostFunction(
      const CameraConstPtr &camera, const Eigen::Quaterniond &cam_odo_q,
      const Eigen::Vector3d &cam_odo_t, const Eigen::Vector3d &odo_pos,
      const Eigen::Vector3d &odo_att, const Eigen::Vector2d &observed_p,
      int flags) const;

  ceres::CostFunction *generateCostFunction(
      const CameraConstPtr &cameraLeft, const CameraConstPtr &cameraRight,
      const Eigen::Vector3d &observed_P, const Eigen::Vector2d &observed_p_left,
      const Eigen::Vector2d &observed_p_right) const;

 private:
  static boost::shared_ptr<CostFunctionFactory> m_instance;
};
}  // namespace camodocal

#endif  // COSTFUNCTIONFACTORY_H
