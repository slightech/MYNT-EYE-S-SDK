#include <memory>

#include "mynteye/logger.h"
#include "mynteye/types.h"

#include "format.hpp"

namespace mynteye_jni {

using RawFormat = MYNTEYE_NAMESPACE::Format;
using JniFormat = mynteye_jni::Format;

inline
RawFormat from_jni(const JniFormat& format) {
  switch (format) {
    case JniFormat::GREY: return RawFormat::GREY;
    case JniFormat::YUYV: return RawFormat::YUYV;
    case JniFormat::BGR888: return RawFormat::BGR888;
    case JniFormat::RGB888: return RawFormat::RGB888;
    default:
      LOG(FATAL) << "Format is unknown";
  }
}

inline
JniFormat to_jni(const RawFormat& format) {
  switch (format) {
    case RawFormat::GREY: return JniFormat::GREY;
    case RawFormat::YUYV: return JniFormat::YUYV;
    case RawFormat::BGR888: return JniFormat::BGR888;
    case RawFormat::RGB888: return JniFormat::RGB888;
    default:
      LOG(FATAL) << "Format is unknown";
  }
}

}  // namespace mynteye_jni
