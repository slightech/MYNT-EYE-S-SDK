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
#ifndef SRC_MYNTEYE_API_CAMERA_MODELS_QUATERNION_H_
#define SRC_MYNTEYE_API_CAMERA_MODELS_QUATERNION_H_

#include <cmath>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

namespace models {
template<typename T>
class Quaternion {
 public:
  Quaternion(void) {}
  explicit Quaternion(SMatrix<double> m) {
    _w = sqrt(m(0) + m(4) + m(8) + 1) / 2;
    _x = (m(5) - m(7)) / (4 * _w);
    _y = (m(6) - m(2)) / (4 * _w);
    _z = (m(1) - m(3)) / (4 * _w);
  }
  Quaternion(T X, T Y, T Z, T W) : _x(X), _y(Y), _z(Z), _w(W) {}
  void normalize(void) {
    double len;
    len = sqrt(_x * _x + _y * _y + _z * _z + _w * _w);
    _x = _x  /  len;
    _y = _y  /  len;
    _z = _z  /  len;
    _w = _w  /  len;
  }
  inline T x() {return _x;}
  inline T y() {return _y;}
  inline T z() {return _z;}
  inline T w() {return _w;}
  SMatrix<double> toRotationMatrix(void) const {
    SMatrix<double> r(3);
    double q0 = _w, q1 = _x, q2 = _y, q3 = _z;
    r(0) = 1  -  2 * q2 * q2 - 2 * q3 * q3;
    r(1) = 2 * q1 * q2 + 2 * q0 * q3;
    r(2) = 2 * q1 * q3 - 2 * q0 * q2;
    r(3) = 2 * q1 * q2 - 2 * q0 * q3;
    r(4) = 1 - 2 * q1 * q1 - 2 * q3 * q3;
    r(5) = 2 * q2 * q3 + 2 * q0 * q1;
    r(6) = 2 * q1 * q3 + 2 * q0 * q2;
    r(7) = 2 * q2 * q3 - 2 * q0 * q1;
    r(8) = 1 - 2 * q1 * q1 - 2 * q2 * q2;
    return r;
  }

 private:
  T _x;
  T _y;
  T _z;
  T _w;
};
}   // namespace models

MYNTEYE_END_NAMESPACE
#endif   // SRC_MYNTEYE_API_CAMERA_MODELS_QUATERNION_H_
