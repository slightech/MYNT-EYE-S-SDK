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
#include <camodocal/sparse_graph/Transform.h>

namespace camodocal {

Transform::Transform() {
  m_q.setIdentity();
  m_t.setZero();
}

Transform::Transform(const Eigen::Matrix4d &H) {
  m_q = Eigen::Quaterniond(H.block<3, 3>(0, 0));
  m_t = H.block<3, 1>(0, 3);
}

Eigen::Quaterniond &Transform::rotation(void) {
  return m_q;
}

const Eigen::Quaterniond &Transform::rotation(void) const {
  return m_q;
}

double *Transform::rotationData(void) {
  return m_q.coeffs().data();
}

const double *const Transform::rotationData(void) const {
  return m_q.coeffs().data();
}

Eigen::Vector3d &Transform::translation(void) {
  return m_t;
}

const Eigen::Vector3d &Transform::translation(void) const {
  return m_t;
}

double *Transform::translationData(void) {
  return m_t.data();
}

const double *const Transform::translationData(void) const {
  return m_t.data();
}

Eigen::Matrix4d Transform::toMatrix(void) const {
  Eigen::Matrix4d H;
  H.setIdentity();
  H.block<3, 3>(0, 0) = m_q.toRotationMatrix();
  H.block<3, 1>(0, 3) = m_t;

  return H;
}
}  // namespace camodocal
