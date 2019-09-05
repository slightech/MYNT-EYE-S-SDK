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
#ifndef SRC_MYNTEYE_API_CAMERA_MODELS_CTAINBASE_H_
#define SRC_MYNTEYE_API_CAMERA_MODELS_CTAINBASE_H_

#include "util/matrix.h"
#include "util/matrix_square.h"
#include "util/matrix_solver.h"
#include "util/quaternion.h"

#include "mynteye/mynteye.h"

// #define _DOUTPUT
MYNTEYE_BEGIN_NAMESPACE

namespace models {
  typedef SMatrix<double> Matrixd;
  typedef Matrix<double> MatrixXd;
  typedef Matrix<double> Matrix23d;
  typedef SMatrix<double> Matrix3d;
  typedef SMatrix<double> Matrix4d;

  typedef SMatrix<float> Matrixf;
  typedef Matrixf Matrix2f;
  typedef Matrixf Matrix3f;

  typedef Matrix<float> Vectorf;
  typedef Vectorf Vector2f;
  typedef Vectorf Vector3f;

  typedef Matrix<double> Vectord;
  typedef Matrix<double> Vector2d;
  typedef Matrix<double> Vector3d;
  typedef Matrix<double> MatrixXcd;

  typedef Quaternion<double> Quaterniond;
}   // namespace models

MYNTEYE_END_NAMESPACE
#endif  // SRC_MYNTEYE_API_CAMERA_MODELS_CTAINBASE_H_
