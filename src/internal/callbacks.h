#ifndef MYNTEYE_INTERNAL_CALLBACKS_H_  // NOLINT
#define MYNTEYE_INTERNAL_CALLBACKS_H_
#pragma once

#include <functional>

#include "mynteye/mynteye.h"
#include "mynteye/types.h"

MYNTEYE_BEGIN_NAMESPACE

namespace device {

struct MYNTEYE_API StreamData {
  ImgData img;
};

struct MYNTEYE_API MotionData {
  ImuData imu;
};

using StreamCallback = std::function<void(const StreamData &data)>;
using MotionCallback = std::function<void(const MotionData &data)>;

}  // namespace device

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_CALLBACKS_H_ NOLINT
