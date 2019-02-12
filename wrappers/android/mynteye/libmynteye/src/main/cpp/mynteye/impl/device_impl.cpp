#include "device_impl.hpp"

#include "mynteye/device/context.h"
#include "mynteye/device/device.h"
#include "mynteye/logger.h"

#include "device_usb_info.hpp"
#include "stream_request.hpp"
#include "type_conversion.hpp"

#include "frame_impl.hpp"
#include "motion_data_impl.hpp"
#include "stream_data_impl.hpp"

#include "mynteye/uvc/uvc.h"
#include "internal/uvc_device.h"

MYNTEYE_USE_NAMESPACE

namespace mynteye_jni {

/*
std::vector<::mynteye_jni::DeviceUsbInfo> Device::Query() {
  VLOG(2) << __func__;
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

std::shared_ptr<Device> Device::Create(const ::mynteye_jni::DeviceUsbInfo & info) {
  VLOG(2) << __func__;
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
*/

std::shared_ptr<Device> Device::Create(const ::mynteye_jni::DeviceUsbInfo & info) {
  VLOG(2) << __func__;
  auto device = uvc::create_device(from_jni(info));
  auto name = uvc::get_name(*device);
  auto vid = uvc::get_vendor_id(*device);
  auto pid = uvc::get_product_id(*device);
  VLOG(2) << "UVC device detected, name: " << name << ", vid: 0x" << std::hex
          << vid << ", pid: 0x" << std::hex << pid;
  return std::make_shared<DeviceImpl>(
      MYNTEYE_NAMESPACE::Device::Create(name, device));
}

DeviceImpl::DeviceImpl(const device_t & device) : Device(), device_(device) {
  VLOG(2) << __func__;
}

DeviceImpl::~DeviceImpl() {
  VLOG(2) << __func__;
}

std::vector<::mynteye_jni::StreamRequest> DeviceImpl::GetStreamRequests() {
  VLOG(2) << __func__;
  std::vector<::mynteye_jni::StreamRequest> requests;

  int32_t i = 0;
  for (auto&& req : device_->GetStreamRequests()) {
    requests.emplace_back(i,
        req.width, req.height, to_jni(req.format), req.fps);
    ++i;
  }

  return requests;
}

void DeviceImpl::ConfigStreamRequest(
    const ::mynteye_jni::StreamRequest & request) {
  VLOG(2) << __func__;
  int32_t i = 0;
  for (auto&& req : device_->GetStreamRequests()) {
    if (i == request.index) {
      device_->ConfigStreamRequest(req);
      return;
    }
    ++i;
  }
}

void DeviceImpl::Start(::mynteye_jni::Source source) {
  device_->Start(from_jni(source));
}

void DeviceImpl::Stop(::mynteye_jni::Source source) {
  device_->Stop(from_jni(source));
}

void DeviceImpl::WaitForStreams() {
  device_->WaitForStreams();
}

std::shared_ptr<::mynteye_jni::StreamData> DeviceImpl::GetStreamData(
    ::mynteye_jni::Stream stream) {
  auto&& data = device_->GetStreamData(from_jni(stream));
  return std::make_shared<StreamDataImpl>(data);
}

std::vector<std::shared_ptr<::mynteye_jni::StreamData>>
DeviceImpl::GetStreamDatas(::mynteye_jni::Stream stream) {
  std::vector<std::shared_ptr<::mynteye_jni::StreamData>> datas;
  for (auto&& data : device_->GetStreamDatas(from_jni(stream))) {
    datas.push_back(std::make_shared<StreamDataImpl>(data));
  }
  return datas;
}

void DeviceImpl::EnableMotionDatas(int32_t max_size) {
  device_->EnableMotionDatas(max_size);
}

std::vector<std::shared_ptr<::mynteye_jni::MotionData>>
DeviceImpl::GetMotionDatas() {
  std::vector<std::shared_ptr<::mynteye_jni::MotionData>> datas;
  for (auto&& data : device_->GetMotionDatas()) {
    datas.push_back(std::make_shared<MotionDataImpl>(data));
  }
  return datas;
}

}  // namespace mynteye_jni
