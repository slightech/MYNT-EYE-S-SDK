#ifndef MYNTEYE_TYPES_H_  // NOLINT
#define MYNTEYE_TYPES_H_
#pragma once

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

template <class T>
class big_endian {
  T be_value;

 public:
  operator T() const {  // convert to T from big to little endian
    T le_value = 0;
    for (unsigned int i = 0; i < sizeof(T); ++i) {
      reinterpret_cast<char *>(&le_value)[i] =
          reinterpret_cast<const char *>(&be_value)[sizeof(T) - i - 1];
    }
    return le_value;
  }
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_TYPES_H_ NOLINT
