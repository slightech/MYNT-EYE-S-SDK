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
#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <boost/shared_ptr.hpp>
#include "eigen3/Eigen/Dense"
#include <stdint.h>

namespace camodocal {

class Transform {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Transform();
  Transform(const Eigen::Matrix4d &H);

  Eigen::Quaterniond &rotation(void);
  const Eigen::Quaterniond &rotation(void) const;
  double *rotationData(void);
  const double *const rotationData(void) const;

  Eigen::Vector3d &translation(void);
  const Eigen::Vector3d &translation(void) const;
  double *translationData(void);
  const double *const translationData(void) const;

  Eigen::Matrix4d toMatrix(void) const;

 private:
  Eigen::Quaterniond m_q;
  Eigen::Vector3d m_t;
};
}  // namespace camodocal

#endif  // TRANSFORM_H
