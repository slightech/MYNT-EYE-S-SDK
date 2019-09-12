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
#ifndef SRC_MYNTEYE_API_CAMERA_MODELS_SQUAREMATRIX_H_
#define SRC_MYNTEYE_API_CAMERA_MODELS_SQUAREMATRIX_H_

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

namespace models {
#define Matrix_ Matrix<_Scalar>
template<typename _Scalar>
class SMatrix: public Matrix_{
 public:
  explicit SMatrix(int dim) : Matrix_(dim, dim) {}
  SMatrix() : Matrix_(0, 0) {}
  SMatrix(_Scalar _data[], int dim) : Matrix_(_data, dim, dim) {}
  SMatrix(_Scalar **_data, int dim) : Matrix_(_data, dim, dim) {}
  explicit SMatrix(Matrix_ m) : Matrix_(m) {}
  _Scalar determinant(void) const;
  _Scalar M(int m, int n) const;
  SMatrix<_Scalar> inverse(void) const;
  void operator =(Matrix<_Scalar> m) {
    SMatrix t(m);
    *this = t;
  }
};

template<typename _Scalar>
SMatrix<_Scalar> SMatrix<_Scalar>::inverse(void) const {
    SMatrix<_Scalar> res(Matrix_::_Rows);
    _Scalar d = determinant();
    for (int i = 0; i < Matrix_::_Rows; i++) {
        for (int j = 0; j < Matrix_::_Cols; j++) {
            res.Data(j, i) = 1.0 * M(i, j) / d;
        }
    }
    return res;
}

template<typename _Scalar>
_Scalar SMatrix<_Scalar>::determinant(void) const {
  int r, c, m;
  int lop = 0;
  int n = Matrix_::_Rows;
  _Scalar result = 0;
  _Scalar mid = 1;
  if (n != 1) {
    lop = (n == 2) ? 1 : n;
    for (m = 0; m < lop; m++) {
        mid = 1;
        for (r = 0, c = m; r < n; r++, c++) {
            mid = mid * (*(Matrix_::data+r*n+c%n));
        }
        result += mid;
    }
    for (m = 0; m < lop; m++) {
        mid = 1;
        for (r = 0, c = n-1-m+n; r < n; r++, c--) {
            mid = mid * (*(Matrix_::data + r * n + c % n));
        }
        result -= mid;
    }
  } else {
    result = Matrix_::data[0];
  }
  return result;
}

template<typename _Scalar>
_Scalar SMatrix<_Scalar>::M(int m, int n) const {
  float mid_result = 0;
  int sign = 1;
  int k = Matrix_::_Rows;
  SMatrix mid(k - 1);
  int c = 0;
  for (int i = 0; i < k; i++) {
    for (int j = 0; j < k; j++) {
      if (i != m && j != n) {
        mid.Data(c++) = Matrix_::cData(i, j);
      }
    }
  }
  sign = (m+n)%2 == 0 ? 1 : -1;
  mid_result = static_cast<_Scalar>(sign) * mid.determinant();
  return mid_result;
}
#undef Matrix_

}   //  namespace models

MYNTEYE_END_NAMESPACE
#endif  // SRC_MYNTEYE_API_CAMERA_MODELS_SQUAREMATRIX_H_
