#ifndef MYNTEYE_DEVICE_H_  // NOLINT
#define MYNTEYE_DEVICE_H_
#pragma once

#include <memory>
#include <string>

#include "mynteye/mynteye.h"

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

struct device;

}  // namespace uvc

class Device {
 public:
  Device();
  virtual ~Device();

  static std::shared_ptr<Device> Create(
      const std::string &name, std::shared_ptr<uvc::device> device);

 private:
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_H_ NOLINT
