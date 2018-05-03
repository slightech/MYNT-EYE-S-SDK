#include "api/api.h"

#include <glog/logging.h>

#include <thread>

#include "mynteye/glog_init.h"
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

std::shared_ptr<API> API::Create(int argc, char *argv[]) {
  static glog_init _(argc, argv);
  return std::make_shared<API>(device::select());
}

std::shared_ptr<API> API::Create(
    int argc, char *argv[], std::shared_ptr<Device> device) {
  static glog_init _(argc, argv);
  return std::make_shared<API>(device);
}

Model API::GetModel() const {
  return device_->GetModel();
}

bool API::Supports(const Stream &stream) const {
  return synthetic_->Supports(stream);
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

void API::SetStreamCallback(const Stream &stream, stream_callback_t callback) {
  synthetic_->SetStreamCallback(stream, callback);
}

void API::SetMotionCallback(motion_callback_t callback) {
  static auto callback_ = callback;
  if (callback_) {
    device_->SetMotionCallback(
        [](const device::MotionData &data) { callback_({data.imu}); });
  } else {
    device_->SetMotionCallback(nullptr);
  }
}

bool API::HasStreamCallback(const Stream &stream) const {
  return synthetic_->HasStreamCallback(stream);
}

bool API::HasMotionCallback() const {
  return device_->HasMotionCallback();
}

void API::Start(const Source &source) {
  if (source == Source::VIDEO_STREAMING) {
    synthetic_->StartVideoStreaming();
  } else if (source == Source::MOTION_TRACKING) {
    device_->StartMotionTracking();
  } else if (source == Source::ALL) {
    Start(Source::VIDEO_STREAMING);
    Start(Source::MOTION_TRACKING);
  } else {
    LOG(FATAL) << "Unsupported source :(";
  }
}

void API::Stop(const Source &source) {
  if (source == Source::VIDEO_STREAMING) {
    synthetic_->StopVideoStreaming();
  } else if (source == Source::MOTION_TRACKING) {
    device_->StopMotionTracking();
  } else if (source == Source::ALL) {
    Stop(Source::MOTION_TRACKING);
    // Must stop motion tracking before video streaming and sleep a moment here
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    Stop(Source::VIDEO_STREAMING);
  } else {
    LOG(FATAL) << "Unsupported source :(";
  }
}

void API::WaitForStreams() {
  synthetic_->WaitForStreams();
}

void API::EnableStreamData(const Stream &stream) {
  synthetic_->EnableStreamData(stream);
}

void API::DisableStreamData(const Stream &stream) {
  synthetic_->DisableStreamData(stream);
}

api::StreamData API::GetStreamData(const Stream &stream) {
  return synthetic_->GetStreamData(stream);
}

std::vector<api::StreamData> API::GetStreamDatas(const Stream &stream) {
  return synthetic_->GetStreamDatas(stream);
}

void API::EnableMotionDatas(std::size_t max_size) {
  device_->EnableMotionDatas(max_size);
}

std::vector<api::MotionData> API::GetMotionDatas() {
  std::vector<api::MotionData> datas;
  for (auto &&data : device_->GetMotionDatas()) {
    datas.push_back({data.imu});
  }
  return datas;
}

std::shared_ptr<Device> API::device() {
  return device_;
}

MYNTEYE_END_NAMESPACE
