#pragma once

#include <memory>

#include "mynteye/logger.h"
#include "mynteye/types.h"

#include "format.hpp"
#include "model.hpp"
#include "source.hpp"
#include "stream.hpp"

namespace mynteye_jni {

using RawModel = MYNTEYE_NAMESPACE::Model;
using JniModel = mynteye_jni::Model;

inline
RawModel from_jni(const JniModel& model) {
  switch (model) {
    case JniModel::STANDARD: return RawModel::STANDARD;
    case JniModel::STANDARD2: return RawModel::STANDARD2;
    case JniModel::STANDARD210A: return RawModel::STANDARD210A;
    default:
      LOG(FATAL) << "Model is unknown";
  }
}

inline
JniModel to_jni(const RawModel& model) {
  switch (model) {
    case RawModel::STANDARD: return JniModel::STANDARD;
    case RawModel::STANDARD2: return JniModel::STANDARD2;
    case RawModel::STANDARD210A: return JniModel::STANDARD210A;
    default:
      LOG(FATAL) << "Model is unknown";
  }
}

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

using RawSource = MYNTEYE_NAMESPACE::Source;
using JniSource = mynteye_jni::Source;

inline
RawSource from_jni(const JniSource& source) {
  switch (source) {
    case JniSource::VIDEO_STREAMING: return RawSource::VIDEO_STREAMING;
    case JniSource::MOTION_TRACKING: return RawSource::MOTION_TRACKING;
    case JniSource::ALL: return RawSource::ALL;
    default:
      LOG(FATAL) << "Source is unknown";
  }
}

inline
JniSource to_jni(const RawSource& source) {
  switch (source) {
    case RawSource::VIDEO_STREAMING: return JniSource::VIDEO_STREAMING;
    case RawSource::MOTION_TRACKING: return JniSource::MOTION_TRACKING;
    case RawSource::ALL: return JniSource::ALL;
    default:
      LOG(FATAL) << "Source is unknown";
  }
}

using RawStream = MYNTEYE_NAMESPACE::Stream;
using JniStream = mynteye_jni::Stream;

inline
RawStream from_jni(const JniStream& stream) {
  switch (stream) {
    case JniStream::LEFT: return RawStream::LEFT;
    case JniStream::RIGHT: return RawStream::RIGHT;
    default:
      LOG(FATAL) << "Stream is unknown";
  }
}

inline
JniStream to_jni(const RawStream& stream) {
  switch (stream) {
    case RawStream::LEFT: return JniStream::LEFT;
    case RawStream::RIGHT: return JniStream::RIGHT;
    default:
      LOG(FATAL) << "Stream is unknown";
  }
}

}  // namespace mynteye_jni
