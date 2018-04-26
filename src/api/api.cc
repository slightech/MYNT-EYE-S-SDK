#include "api/api.h"

#include <glog/logging.h>

#include "mynteye/utils.h"

#include "device/device.h"

MYNTEYE_BEGIN_NAMESPACE

API::API(std::shared_ptr<Device> device) : device_(device) {
  VLOG(2) << __func__;
}

API::~API() {
  VLOG(2) << __func__;
}

std::shared_ptr<API> API::Create() {
  return Create(device::select());
}

std::shared_ptr<API> API::Create(std::shared_ptr<Device> device) {
  return std::make_shared<API>(device);
}

MYNTEYE_END_NAMESPACE
