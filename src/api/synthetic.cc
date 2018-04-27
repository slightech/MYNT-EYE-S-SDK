#include "api/synthetic.h"

#include <glog/logging.h>

#include <stdexcept>

#include "api/processor/depth_processor.h"
#include "api/processor/disparity_normalized_processor.h"
#include "api/processor/disparity_processor.h"
#include "api/processor/points_processor.h"
#include "api/processor/processor.h"
#include "api/processor/rectify_processor.h"
#include "device/device.h"

MYNTEYE_BEGIN_NAMESPACE

Synthetic::Synthetic(API *api) : api_(api) {
  VLOG(2) << __func__;
  CHECK_NOTNULL(api_);
  InitStreamSupports();
}

Synthetic::~Synthetic() {
  VLOG(2) << __func__;
}

Synthetic::mode_t Synthetic::GetMode(const Stream &stream) const {
  try {
    return stream_supports_mode_.at(stream);
  } catch (const std::out_of_range &e) {
    return MODE_LAST;
  }
}

bool Synthetic::Supports(const Stream &stream) const {
  return GetMode(stream) != MODE_LAST;
}

void Synthetic::EnableStreamData(const Stream &stream) {
  if (Supports(stream))
    return;
  switch (stream) {
    case Stream::LEFT_RECTIFIED: {
      if (Supports(Stream::LEFT)) {
        stream_supports_mode_[stream] = MODE_SYNTHETIC;
        return;
      }
    } break;
    case Stream::RIGHT_RECTIFIED: {
      if (Supports(Stream::RIGHT)) {
        stream_supports_mode_[stream] = MODE_SYNTHETIC;
        return;
      }
    } break;
    case Stream::DISPARITY: {
      if (Supports(Stream::LEFT) && Supports(Stream::RIGHT)) {
        stream_supports_mode_[stream] = MODE_SYNTHETIC;
        return;
      }
    } break;
    case Stream::DISPARITY_NORMALIZED: {
      if ((Supports(Stream::LEFT) && Supports(Stream::RIGHT)) ||
          Supports(Stream::DISPARITY)) {
        stream_supports_mode_[stream] = MODE_SYNTHETIC;
        return;
      }
    } break;
    case Stream::DEPTH: {
      if ((Supports(Stream::LEFT) && Supports(Stream::RIGHT)) ||
          Supports(Stream::DISPARITY) || Supports(Stream::POINTS)) {
        stream_supports_mode_[stream] = MODE_SYNTHETIC;
        return;
      }
    } break;
    case Stream::POINTS: {
      if ((Supports(Stream::LEFT) && Supports(Stream::RIGHT)) ||
          Supports(Stream::DISPARITY)) {
        stream_supports_mode_[stream] = MODE_SYNTHETIC;
        return;
      }
    } break;
    default:
      break;
  }
  LOG(WARNING) << "Enable stream data of " << stream << " failed";
}

void Synthetic::DisableStreamData(const Stream &stream) {
  UNUSED(stream)
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

void Synthetic::InitStreamSupports() {
  auto &&device = api_->device();
  for (Stream s = Stream::LEFT; s < Stream::LAST;) {
    if (device->Supports(s)) {
      stream_supports_mode_[s] = MODE_NATIVE;
    }
    s = static_cast<Stream>(static_cast<std::uint8_t>(s) + 1);
  }
}

MYNTEYE_END_NAMESPACE
