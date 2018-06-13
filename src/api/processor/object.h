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
#ifndef MYNTEYE_OBJECT_H_  // NOLINT
#define MYNTEYE_OBJECT_H_
#pragma once

#include <opencv2/core/core.hpp>

#include <memory>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

/**
 * Input & output object.
 */
struct MYNTEYE_API Object {
  Object() = default;
  virtual ~Object() = default;

  virtual Object *Clone() const = 0;

  /** Cast the obj to T pointer */
  template <typename T>
  static T *Cast(Object *obj) {
    return dynamic_cast<T *>(obj);
  }

  /** Cast the obj to const T pointer */
  template <typename T>
  static const T *Cast(const Object *obj) {
    return dynamic_cast<const T *>(obj);
  }

  template <typename T>
  static std::shared_ptr<T> Cast(const std::shared_ptr<Object> &obj) {
    return std::dynamic_pointer_cast<T>(obj);
  }
};

/**
 * Input & output object of one cv::Mat.
 */
struct MYNTEYE_API ObjMat : public Object {
  ObjMat() = default;
  explicit ObjMat(const cv::Mat &value) : value(value) {}

  /** The value */
  cv::Mat value;

  Object *Clone() const {
    ObjMat *mat = new ObjMat;
    mat->value = value.clone();
    return mat;
  }
};

/**
 * Input & output object of two cv::Mat.
 */
struct MYNTEYE_API ObjMat2 : public Object {
  ObjMat2() = default;
  ObjMat2(const cv::Mat &first, const cv::Mat &second)
      : first(first), second(second) {}

  /** The first value */
  cv::Mat first;

  /** The second value */
  cv::Mat second;

  Object *Clone() const {
    ObjMat2 *mat2 = new ObjMat2;
    mat2->first = first.clone();
    mat2->second = second.clone();
    return mat2;
  }
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_OBJECT_H_  NOLINT
