#include "internal/streams.h"

#include <glog/logging.h>

#include <algorithm>

MYNTEYE_BEGIN_NAMESPACE

Streams::Streams()
    : stream_capabilities_(
          {Capabilities::STEREO, Capabilities::COLOR, Capabilities::DEPTH,
           Capabilities::POINTS, Capabilities::FISHEYE, Capabilities::INFRARED,
           Capabilities::INFRARED2}) {
  VLOG(2) << __func__;
}

Streams::~Streams() {
  VLOG(2) << __func__;
}

void Streams::ConfigStream(
    const Capabilities &capability, const StreamRequest &request) {
  if (!IsStreamCapability(capability)) {
    LOG(FATAL) << "Cannot config stream without stream capability";
  }
  stream_config_requests_[capability] = request;
}

void Streams::PushStream(const Capabilities &capability, const void *data) {
  if (!HasStreamConfigRequest(capability)) {
    LOG(FATAL) << "Cannot push stream without stream config request";
  }
  auto frame =
      std::make_shared<frame_t>(GetStreamConfigRequest(capability), data);
  // stream_datas_map_[Stream::LEFT];
  // stream_datas_map_[Stream::RIGHT];
}

bool Streams::IsStreamCapability(const Capabilities &capability) const {
  return std::find(
             stream_capabilities_.begin(), stream_capabilities_.end(),
             capability) != stream_capabilities_.end();
}

bool Streams::HasStreamConfigRequest(const Capabilities &capability) const {
  return stream_config_requests_.find(capability) !=
         stream_config_requests_.end();
}

const StreamRequest &Streams::GetStreamConfigRequest(
    const Capabilities &capability) const {
  return stream_config_requests_.at(capability);
}

bool Streams::HasStreamDatas(const Stream &stream) const {
  return stream_datas_map_.find(stream) != stream_datas_map_.end();
}

Streams::stream_datas_t &Streams::GetStreamDatas(const Stream &stream) {
  return stream_datas_map_[stream];
}

MYNTEYE_END_NAMESPACE
