#include "api/synthetic.h"

#include <glog/logging.h>

#include "api/processor/processor.h"
#include "device/device.h"

MYNTEYE_BEGIN_NAMESPACE

Synthetic::Synthetic(API *api) : api_(api) {
  VLOG(2) << __func__;
}

Synthetic::~Synthetic() {
  VLOG(2) << __func__;
}

bool Synthetic::Supports(const Stream &stream) const {
  return api_->device()->Supports(stream);
}

void Synthetic::SetStreamCallback(
    const Stream &stream, stream_callback_t callback) {
  UNUSED(stream)
  UNUSED(callback)
}

bool Synthetic::HasStreamCallback(const Stream &stream) const {
  UNUSED(stream)
  return false;
}

void Synthetic::StartVideoStreaming() {}

void Synthetic::StopVideoStreaming() {}

void Synthetic::WaitForStreams() {}

api::StreamData Synthetic::GetStreamData(const Stream &stream) {
  UNUSED(stream)
  return {};
}

std::vector<api::StreamData> Synthetic::GetStreamDatas(const Stream &stream) {
  UNUSED(stream)
  return {};
}

MYNTEYE_END_NAMESPACE
