#ifndef MYNTEYE_DEVICE_S_H_  // NOLINT
#define MYNTEYE_DEVICE_S_H_
#pragma once

#include <memory>

#include "device/device.h"

MYNTEYE_BEGIN_NAMESPACE

class StandardDevice : public Device {
 public:
  explicit StandardDevice(std::shared_ptr<uvc::device> device);
  virtual ~StandardDevice();
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_S_H_ NOLINT
