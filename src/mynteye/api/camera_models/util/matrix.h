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
#ifndef SRC_MYNTEYE_API_CAMERA_MODELS_MATRIX_H_
#define SRC_MYNTEYE_API_CAMERA_MODELS_MATRIX_H_

#include <cstring>
#include <iostream>
#include <cmath>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

namespace models {
template<typename _Scalar>
class Matrix {
 public:
  Matrix(int _rows, int _cols) :
    _Rows(_rows), _Cols(_cols), _isSub(0), input_id(0) {
  _startRow = 0;
  _startCol = 0;
  _Rows_raw = _rows;
  _Cols_raw = _cols;
  data = new _Scalar[_Rows * _Cols];
  memset(data, 0, _Rows * _Cols * sizeof(_Scalar));
  }
  Matrix(void) :
    _Rows(0), _Cols(0), _isSub(0), input_id(0) {
  _startRow = 0;
  _startCol = 0;
  _Rows_raw = 0;
  _Cols_raw = 0;
  }
  Matrix(_Scalar _data[], int _rows, int _cols) :
      _Rows(_rows), _Cols(_cols), _isSub(0), input_id(0) {
    _startRow = 0;
    _startCol = 0;
    _Rows_raw = _rows;
    _Cols_raw = _cols;
    data = new _Scalar[_rows * _Cols];
    memcpy(data, _data, _Rows * _Cols * sizeof(_Scalar));
  }

  Matrix(_Scalar **_data, int _rows, int _cols) :
      _Rows(_rows), _Cols(_cols), _isSub(0), input_id(0) {
    _startRow = 0;
    _startCol = 0;
    _Rows_raw = _rows;
    _Cols_raw = _cols;
    data = new _Scalar[_rows * _Cols];
    for (int i = 0; i < _Rows; ++i) {
    memcpy(data + i * _Cols, *(_data + i), _Cols * sizeof(_Scalar));
    }
  }

  template<typename T>
  Matrix<T> cast(void) const {
    Matrix<T> res(_Rows, _Cols);
    for (int i = 0; i < _Rows; i++) {
      for (int j = 0; j < _Cols; j++) {
        res(i, j) = cData(i, j);
      }
    }
    return res;
  }

  void setIdentity(void) {
    for (int i = 0; i < _Rows; i++) {
      for (int j = 0; j < _Cols; j++) {
        if (i == j) {
          Data(i, j) = 1;
        } else {
          Data(i, j) = 0;
        }
      }
    }
  }
  void setZero(void) {
    for (int i = 0; i < _Rows; i++) {
      for (int j = 0; j < _Cols; j++) {
        Data(i, j) = 0;
      }
    }
  }
  inline int cols(void) const { return _Cols; }

  inline int rows(void) const { return _Rows; }

  inline int size(void) const { return cols() * rows(); }
  inline _Scalar * addr(void) {
    return data;
  }

  friend Matrix<_Scalar> &operator <<(Matrix<_Scalar> &m, _Scalar val) {
    m.Data(m.input_id++) = val;
    return m;
  }

  friend std::ostream &operator <<(std::ostream &os,
      const Matrix<_Scalar> &m) {
    for (int i = 0; i < m._Rows; i++) {
      for (int j = 0; j < m._Cols - 1; j++) {
        std::cout.width(10);
        std::cout.setf(std::ios::left);
        std::cout.precision(6);
        os << m.cData(i, j);
      }
      std::cout.width(9);
      std::cout.setf(std::ios::left);
      std::cout.precision(8);
      os << m.cData(i, m._Cols - 1) << std::endl;
    }
    return os;
  }

  friend Matrix<_Scalar> operator *(
      double a, const Matrix<_Scalar> &m) {
    Matrix<_Scalar> res;
    res = m;
    for (int i = 0; i < m._Rows; i++) {
      for (int j = 0; j < m._Cols; j++) {
        res.Data(i, j) *= a;
      }
    }
    return res;
  }

  friend Matrix<_Scalar> operator *(
      const Matrix<_Scalar> &m, double a) {
    Matrix<_Scalar> res;
    res = m;
    for (int i = 0; i < m._Rows; i++) {
      for (int j = 0; j < m._Cols; j++) {
        res.Data(i, j) *= a;
      }
    }
    return res;
  }

  friend Matrix<_Scalar> operator -(
    const Matrix<_Scalar> &m) {
    Matrix<_Scalar> res;
    res = m;
    for (int i = 0; i < m._Rows; i++) {
      for (int j = 0; j < m._Cols; j++) {
        res.Data(i, j) *= -1;
      }
    }
    return res;
  }

  friend Matrix<_Scalar> operator +(
    double a, const Matrix<_Scalar> &m) {
    Matrix<_Scalar> res;
    res = m;
    for (int i = 0; i < m._Rows; i++) {
      for (int j = 0; j < m._Cols; j++) {
        res.Data(i, j) += a;
      }
    }
    return res;
  }

  virtual void operator =(Matrix<_Scalar> m);
  Matrix<_Scalar> operator +(const Matrix<_Scalar> &m) const;
  Matrix<_Scalar> operator -(const Matrix<_Scalar> &m) const;
  Matrix<_Scalar> operator *(const Matrix<_Scalar> &m) const;

  Matrix<_Scalar> operator /(double m) const;
  _Scalar &operator()(int i, int j) {
    return Data(i, j);
  }
  _Scalar &operator()(int id) {
    return Data(id);
  }
  _Scalar operator()(int id) const {
    return cData(id);
  }
  Matrix<_Scalar> transpose() const;

  Matrix<_Scalar> col(int Col) {
    return block(0, Col, _Rows, 1);
  }
  Matrix<_Scalar> row(int Row) {
    return block(Row, 0, 1, _Cols);
  }
  Matrix<_Scalar> block(int sRow, int sCol, int _rows, int _cols) {
    Matrix<_Scalar> sub;
    sub = *this;
    sub.setSub(sRow, sCol, _rows, _cols, data);
    return sub;
  }

  template<int _rows, int _cols>
  Matrix<_Scalar> topLeftCorner(void) {
    Matrix<_Scalar> sub;
    sub = *this;
    sub.setSub(0, 0, _rows, _cols, data);
    return sub;
  }

  template<int _rows, int _cols>
  Matrix<_Scalar> topRightCorner(void) {
    Matrix<_Scalar> sub;
    sub = *this;
    sub.setSub(0, _Cols-_cols, _rows, _cols, data);
    return sub;
  }

  void setSub(int sRow, int sCol, int _rows, int _cols, _Scalar *Data) {
    _isSub = true;
    _Rows_raw = _Rows;
    _Cols_raw = _Cols;
    _Rows = _rows;
    _Cols = _cols;
    _startRow = sRow;
    _startCol = sCol;
    data = Data;
  }

  void normalize(void);
  double norm(void) const;

  virtual ~Matrix() {
    if  (!data)
      delete[] data;
  }

  inline _Scalar *Data(void) {
    return data;
  }

 protected:
  _Scalar *data;
  int _Rows;
  int _Cols;
  bool _isSub;
  int _startRow;
  int _startCol;
  int _Rows_raw;
  int _Cols_raw;
  int input_id;
  inline int id(int i, int j) const {
    if (_isSub)
      return (i + _startRow) * _Cols_raw + j + _startCol;
    else
      return i * _Cols + j;
  }
  inline _Scalar &Data(int i, int j) {
    return data[id(i, j)];
  }
  inline _Scalar &Data(int id) {
    int i = id / _Cols;
    int j = id % _Cols;
    int index;
    if (_isSub)
      index = (i + _startRow) * _Cols_raw + j + _startCol;
    else
      index = i * _Cols + j;
    return data[index];
  }
  inline _Scalar cData(int id) const {
    int i = id / _Cols;
    int j = id % _Cols;
    int index;
    if (_isSub)
      index = (i + _startRow) * _Cols_raw + j + _startCol;
    else
    index = i * _Cols + j;
    return data[index];
  }
  inline _Scalar cData(int i, int j) const {
    return data[id(i, j)];
  }
};    //  class Matrix

template<typename _Scalar>
void Matrix<_Scalar>::operator =(Matrix<_Scalar> m) {
  if (m._isSub) {
    if (_isSub) {
      for (int i = 0; i < m._Rows; i++) {
        for (int j = 0; j < m._Cols; j++) {
          Data(i, j) = m.cData(i, j);
        }
      }
      return;
    }
    _isSub = true;
    _Rows = m._Rows;
    _Cols = m._Cols;
    _Rows_raw = m._Rows_raw;
    _Cols_raw = m._Cols_raw;
    _startRow = m._startRow;
    _startCol = m._startCol;
    data = m.Data();
    return;
  }
  if (!_isSub) {
    if (size() != m.size()) {
      if (size() > 0) {
        delete[] data;
      }
      _Rows = m._Rows;
      _Cols = m._Cols;
      data = new _Scalar[_Rows * _Cols];
    } else {
      _Rows = m._Rows;
      _Cols = m._Cols;
    }
  }

  for (int i = 0; i < m._Rows; i++) {
    for (int j = 0; j < m._Cols; j++) {
      Data(i, j) = m.cData(i, j);
    }
  }
}

template<typename _Scalar>
Matrix<_Scalar> Matrix<_Scalar>::operator +(const Matrix<_Scalar> &m) const {
  Matrix<_Scalar> sum;
  sum = *this;
  for (int i = 0; i < _Rows * _Cols; i++) {
    sum.data[i] += m.data[i];
  }
  return sum;
}

template<typename _Scalar>
Matrix<_Scalar> Matrix<_Scalar>::operator -(const Matrix<_Scalar> &m) const {
  Matrix<_Scalar> sum;
  sum = *this;
  for (int i = 0; i < _Rows * _Cols; i++) {
    sum.data[i] -= m.data[i];
  }
  return sum;
}

template<typename _Scalar>
Matrix<_Scalar> Matrix<_Scalar>::transpose(void) const {
  Matrix<_Scalar> res(_Cols, _Rows);
  for (int i = 0; i < _Rows; i++) {
    for (int j = 0; j < _Cols; j++) {
      res.Data(j, i) = cData(i, j);
    }
  }
  return res;
}

template<typename _Scalar>
Matrix<_Scalar> Matrix<_Scalar>::operator *(const Matrix<_Scalar> &m) const {
  if (_Cols != m._Rows) {
    //  todo:output err
    return m;
  }
  Matrix<_Scalar> res(_Rows, m._Cols);
  for (int i = 0; i < _Rows; i++) {
    for (int j = 0; j < m._Cols; j++) {
      _Scalar sum = 0;
      for (int k = 0; k < _Cols; k++) {
        sum += cData(i, k) * m.cData(k, j);
      }
      res.Data(i, j) = sum;
    }
  }
  return res;
}

template<typename _Scalar>
Matrix<_Scalar> Matrix<_Scalar>::operator /(double m) const {
  Matrix<_Scalar> res;
  res = *this;
  for (int i = 0; i < _Rows; i++) {
    for (int j = 0; j < _Cols; j++) {
      res.Data(i, j) /= m;
    }
  }
  return res;
}


template<typename _Scalar>
void Matrix<_Scalar>::normalize(void) {
  double sum = 0;
  for (int i = 0; i < _Rows; i++) {
    for (int j = 0; j < _Cols; j++) {
      sum += Matrix::cData(i, j);
    }
  }
  sum = sqrt(sum);
  for (int i = 0; i < _Rows; i++) {
    for (int j = 0; j < _Cols; j++) {
      Matrix::Data(i, j) /= sum;
    }
  }
}

template<typename _Scalar>
double Matrix<_Scalar>::norm(void) const {
  double sum = 0;
  for (int i = 0; i < _Rows; i++) {
    for (int j = 0; j < _Cols; j++) {
      sum += Matrix::cData(i, j) * Matrix::cData(i, j);
    }
  }
  sum = sqrt(sum);

  return sum;
}
}   //  namespace models

MYNTEYE_END_NAMESPACE
#endif  //  SRC_MYNTEYE_API_CAMERA_MODELS_MATRIX_H_
