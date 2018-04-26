#include "api/api.h"

#include <glog/logging.h>

#include "mynteye/utils.h"

#include "api/synthetic.h"
#include "device/device.h"

MYNTEYE_BEGIN_NAMESPACE

API::API(std::shared_ptr<Device> device)
    : device_(device), synthetic_(new Synthetic(this)) {
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

Model API::GetModel() const {
  return device_->GetModel();
}

bool API::Supports(const Stream &stream) const {
  return device_->Supports(stream);
}

bool API::Supports(const Capabilities &capability) const {
  return device_->Supports(capability);
}

bool API::Supports(const Option &option) const {
  return device_->Supports(option);
}

const std::vector<StreamRequest> &API::GetStreamRequests(
    const Capabilities &capability) const {
  return device_->GetStreamRequests(capability);
}

void API::ConfigStreamRequest(
    const Capabilities &capability, const StreamRequest &request) {
  device_->ConfigStreamRequest(capability, request);
}

std::string API::GetInfo(const Info &info) const {
  return device_->GetInfo(info);
}

Intrinsics API::GetIntrinsics(const Stream &stream) const {
  return device_->GetIntrinsics(stream);
}

Extrinsics API::GetExtrinsics(const Stream &from, const Stream &to) const {
  return device_->GetExtrinsics(from, to);
}

MotionIntrinsics API::GetMotionIntrinsics() const {
  return device_->GetMotionIntrinsics();
}

Extrinsics API::GetMotionExtrinsics(const Stream &from) const {
  return device_->GetMotionExtrinsics(from);
}

void API::LogOptionInfos() const {
  device_->LogOptionInfos();
}

OptionInfo API::GetOptionInfo(const Option &option) const {
  return device_->GetOptionInfo(option);
}

std::int32_t API::GetOptionValue(const Option &option) const {
  return device_->GetOptionValue(option);
}

void API::SetOptionValue(const Option &option, std::int32_t value) {
  device_->SetOptionValue(option, value);
}

bool API::RunOptionAction(const Option &option) const {
  return device_->RunOptionAction(option);
}

std::shared_ptr<Device> API::device() {
  return device_;
}

MYNTEYE_END_NAMESPACE
