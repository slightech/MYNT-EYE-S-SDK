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
#ifndef EIGENQUATERNIONPARAMETERIZATION_H
#define EIGENQUATERNIONPARAMETERIZATION_H

#include "ceres/local_parameterization.h"

namespace camodocal {

class EigenQuaternionParameterization : public ceres::LocalParameterization {
 public:
  virtual ~EigenQuaternionParameterization() {}
  virtual bool Plus(
      const double *x, const double *delta, double *x_plus_delta) const;
  virtual bool ComputeJacobian(const double *x, double *jacobian) const;
  virtual int GlobalSize() const {
    return 4;
  }
  virtual int LocalSize() const {
    return 3;
  }

 private:
  template <typename T>
  void EigenQuaternionProduct(const T z[4], const T w[4], T zw[4]) const;
};

template <typename T>
void EigenQuaternionParameterization::EigenQuaternionProduct(
    const T z[4], const T w[4], T zw[4]) const {
  zw[0] = z[3] * w[0] + z[0] * w[3] + z[1] * w[2] - z[2] * w[1];
  zw[1] = z[3] * w[1] - z[0] * w[2] + z[1] * w[3] + z[2] * w[0];
  zw[2] = z[3] * w[2] + z[0] * w[1] - z[1] * w[0] + z[2] * w[3];
  zw[3] = z[3] * w[3] - z[0] * w[0] - z[1] * w[1] - z[2] * w[2];
}
}  // namespace camodocal

#endif  // EIGENQUATERNIONPARAMETERIZATION_H
