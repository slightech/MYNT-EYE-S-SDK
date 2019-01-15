#pragma once

#include <vector>
#include <memory>

#include "mynteye/device/device.h"

#include "device.hpp"

namespace mynteye_jni {

class DeviceImpl : public Device {
 public:
  using device_t = std::shared_ptr<MYNTEYE_NAMESPACE::Device>;

  explicit DeviceImpl(const device_t & device);
  ~DeviceImpl();

  std::vector<StreamRequest> GetStreamRequests() override;

  void ConfigStreamRequest(const StreamRequest & request) override;

  void Start() override;

  void Stop() override;

 private:
  device_t device_;
};

}  // namespace mynteye_jni
