#ifndef MYNTEYE_OBJECT_H_  // NOLINT
#define MYNTEYE_OBJECT_H_
#pragma once

#include <opencv2/core/core.hpp>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

/**
 * Input & output object.
 */
struct Object {
  Object() = default;
  virtual ~Object() = default;

  virtual Object *Clone() const = 0;

  template <typename T>
  static T *Cast(Object *obj) {
    return dynamic_cast<T *>(obj);
  }

  template <typename T>
  static const T *Cast(const Object *obj) {
    return dynamic_cast<const T *>(obj);
  }
};

struct ObjMat : public Object {
  ObjMat() = default;
  explicit ObjMat(const cv::Mat &value) : value(value) {}

  cv::Mat value;

  Object *Clone() const {
    ObjMat *mat = new ObjMat;
    mat->value = value.clone();
    return mat;
  }
};

struct ObjMat2 : public Object {
  ObjMat2() = default;
  ObjMat2(const cv::Mat &first, const cv::Mat &second)
      : first(first), second(second) {}

  cv::Mat first;
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
