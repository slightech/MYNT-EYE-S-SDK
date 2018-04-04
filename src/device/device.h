#ifndef MYNTEYE_DEVICE_H_  // NOLINT
#define MYNTEYE_DEVICE_H_
#pragma once

#include <memory>
#include <string>

#include "mynteye/mynteye.h"
#include "mynteye/types.h"

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

struct device;

}  // namespace uvc

struct DeviceInfo;

class Device {
 public:
  Device(const Model &model, std::shared_ptr<uvc::device> device);
  virtual ~Device();

  static std::shared_ptr<Device> Create(
      const std::string &name, std::shared_ptr<uvc::device> device);

  bool Supports(const Stream &stream) const;
  bool Supports(const Capabilities &capability) const;
  bool Supports(const Option &option) const;

  std::shared_ptr<DeviceInfo> GetInfo() const;
  std::string GetInfo(const Info &info) const;

  Model model() const {
    return model_;
  }

 protected:
  std::shared_ptr<uvc::device> device() const {
    return device_;
  }

 private:
  Model model_;
  std::shared_ptr<uvc::device> device_;
  std::shared_ptr<DeviceInfo> device_info_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_H_ NOLINT
