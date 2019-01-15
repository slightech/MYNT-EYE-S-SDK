#include "device_impl.hpp"

#include "mynteye/device/context.h"
#include "mynteye/device/device.h"

#include "device_usb_info.hpp"
#include "stream_request.hpp"
#include "type_conversion.hpp"

MYNTEYE_USE_NAMESPACE

namespace mynteye_jni {

std::vector<DeviceUsbInfo> Device::Query() {
  std::vector<DeviceUsbInfo> infos;

  Context context;
  int32_t i = 0;
  for (auto&& d : context.devices()) {
    infos.emplace_back(i,
      d->GetInfo(Info::DEVICE_NAME),
      d->GetInfo(Info::SERIAL_NUMBER));
    ++i;
  }

  return infos;
}

std::shared_ptr<Device> Device::Create(const DeviceUsbInfo & info) {
  Context context;
  int32_t i = 0;
  for (auto&& d : context.devices()) {
    if (i == info.index) {
      return std::make_shared<DeviceImpl>(d);
    }
    ++i;
  }
  return nullptr;
}

DeviceImpl::DeviceImpl(const device_t & device) : Device(), device_(device) {
}

DeviceImpl::~DeviceImpl() {
}

std::vector<StreamRequest> DeviceImpl::GetStreamRequests() {
  std::vector<StreamRequest> requests;

  int32_t i = 0;
  for (auto&& req : device_->GetStreamRequests()) {
    requests.emplace_back(i,
        req.width, req.height, to_jni(req.format), req.fps);
    ++i;
  }

  return requests;
}

void DeviceImpl::ConfigStreamRequest(const StreamRequest & request) {
  int32_t i = 0;
  for (auto&& req : device_->GetStreamRequests()) {
    if (i == request.index) {
      device_->ConfigStreamRequest(req);
      return;
    }
    ++i;
  }
}

void DeviceImpl::Start() {
}

void DeviceImpl::Stop() {
}

}  // namespace mynteye_jni
