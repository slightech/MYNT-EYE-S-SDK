#include "api/synthetic.h"

#include <glog/logging.h>

#include <algorithm>
#include <functional>
#include <stdexcept>

#include "api/processor/depth_processor.h"
#include "api/processor/disparity_normalized_processor.h"
#include "api/processor/disparity_processor.h"
#include "api/processor/object.h"
#include "api/processor/points_processor.h"
#include "api/processor/processor.h"
#include "api/processor/rectify_processor.h"
#include "device/device.h"

MYNTEYE_BEGIN_NAMESPACE

Synthetic::Synthetic(API *api) : api_(api) {
  VLOG(2) << __func__;
  CHECK_NOTNULL(api_);
  InitStreamSupports();
  InitProcessors();
}

Synthetic::~Synthetic() {
  VLOG(2) << __func__;
  if (processor_) {
    processor_->Deactivate(true);
    processor_ = nullptr;
  }
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
  EnableStreamData(stream, 0);
}

void Synthetic::DisableStreamData(const Stream &stream) {
  DisableStreamData(stream, 0);
}

bool Synthetic::IsStreamDataEnabled(const Stream &stream) const {
  return stream_supports_mode_.find(stream) != stream_supports_mode_.end();
}

void Synthetic::SetStreamCallback(
    const Stream &stream, stream_callback_t callback) {
  if (callback == nullptr) {
    stream_callbacks_.erase(stream);
  } else {
    stream_callbacks_[stream] = callback;
  }
}

bool Synthetic::HasStreamCallback(const Stream &stream) const {
  return stream_callbacks_.find(stream) != stream_callbacks_.end();
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

void Synthetic::EnableStreamData(const Stream &stream, std::uint32_t depth) {
  if (Supports(stream))
    return;
  // Activate processors of synthetic stream
  switch (stream) {
    case Stream::LEFT_RECTIFIED: {
      if (!Supports(Stream::LEFT))
        break;
      stream_supports_mode_[stream] = MODE_SYNTHETIC;
      CHECK(ActivateProcessor<RectifyProcessor>());
    }
      return;
    case Stream::RIGHT_RECTIFIED: {
      if (!Supports(Stream::RIGHT))
        break;
      stream_supports_mode_[stream] = MODE_SYNTHETIC;
      CHECK(ActivateProcessor<RectifyProcessor>());
    }
      return;
    case Stream::DISPARITY: {
      stream_supports_mode_[stream] = MODE_SYNTHETIC;
      EnableStreamData(Stream::LEFT_RECTIFIED, depth + 1);
      EnableStreamData(Stream::RIGHT_RECTIFIED, depth + 1);
      CHECK(ActivateProcessor<DisparityProcessor>());
    }
      return;
    case Stream::DISPARITY_NORMALIZED: {
      stream_supports_mode_[stream] = MODE_SYNTHETIC;
      EnableStreamData(Stream::DISPARITY, depth + 1);
      CHECK(ActivateProcessor<DisparityNormalizedProcessor>());
    }
      return;
    case Stream::POINTS: {
      stream_supports_mode_[stream] = MODE_SYNTHETIC;
      EnableStreamData(Stream::DISPARITY, depth + 1);
      CHECK(ActivateProcessor<PointsProcessor>());
    }
      return;
    case Stream::DEPTH: {
      stream_supports_mode_[stream] = MODE_SYNTHETIC;
      EnableStreamData(Stream::POINTS, depth + 1);
      CHECK(ActivateProcessor<DepthProcessor>());
    }
      return;
    default:
      break;
  }
  if (depth == 0) {
    LOG(WARNING) << "Enable stream data of " << stream << " failed";
  }
}

void Synthetic::DisableStreamData(const Stream &stream, std::uint32_t depth) {
  if (!IsStreamDataEnabled(stream))
    return;
  // Deactivate processors of synthetic stream
  if (stream_supports_mode_[stream] != MODE_NATIVE) {
    stream_supports_mode_.erase(stream);
    switch (stream) {
      case Stream::LEFT_RECTIFIED: {
        if (GetMode(Stream::RIGHT_RECTIFIED) == MODE_SYNTHETIC) {
          DisableStreamData(Stream::RIGHT_RECTIFIED, depth + 1);
        }
        if (GetMode(Stream::DISPARITY) == MODE_SYNTHETIC) {
          DisableStreamData(Stream::DISPARITY, depth + 1);
        }
        DeactivateProcessor<RectifyProcessor>();
      } break;
      case Stream::RIGHT_RECTIFIED: {
        if (GetMode(Stream::LEFT_RECTIFIED) == MODE_SYNTHETIC) {
          DisableStreamData(Stream::LEFT_RECTIFIED, depth + 1);
        }
        if (GetMode(Stream::DISPARITY) == MODE_SYNTHETIC) {
          DisableStreamData(Stream::DISPARITY, depth + 1);
        }
        DeactivateProcessor<RectifyProcessor>();
      } break;
      case Stream::DISPARITY: {
        if (GetMode(Stream::DISPARITY_NORMALIZED) == MODE_SYNTHETIC) {
          DisableStreamData(Stream::DISPARITY_NORMALIZED, depth + 1);
        }
        if (GetMode(Stream::POINTS) == MODE_SYNTHETIC) {
          DisableStreamData(Stream::POINTS, depth + 1);
        }
        DeactivateProcessor<DisparityProcessor>();
      } break;
      case Stream::DISPARITY_NORMALIZED: {
        DeactivateProcessor<DisparityNormalizedProcessor>();
      } break;
      case Stream::POINTS: {
        if (GetMode(Stream::DEPTH) == MODE_SYNTHETIC) {
          DisableStreamData(Stream::DEPTH, depth + 1);
        }
        DeactivateProcessor<PointsProcessor>();
      } break;
      case Stream::DEPTH: {
        DeactivateProcessor<DepthProcessor>();
      } break;
      default:
        return;
    }
    if (depth > 0) {
      LOG(WARNING) << "Disable synthetic stream data of " << stream << " too";
    }
  } else if (depth == 0) {
    LOG(WARNING) << "Disable native stream data of " << stream << " failed";
  }
}

void Synthetic::InitProcessors() {
  auto &&rectify_processor = std::make_shared<RectifyProcessor>();
  auto &&disparity_processor = std::make_shared<DisparityProcessor>();
  auto &&disparitynormalized_processor =
      std::make_shared<DisparityNormalizedProcessor>();
  auto &&points_processor = std::make_shared<PointsProcessor>();
  auto &&depth_processor = std::make_shared<DepthProcessor>();

  using namespace std::placeholders;  // NOLINT
  rectify_processor->SetProcessCallback(
      std::bind(&Synthetic::OnRectifyProcess, this, _1, _2, _3));
  disparity_processor->SetProcessCallback(
      std::bind(&Synthetic::OnDisparityProcess, this, _1, _2, _3));
  disparitynormalized_processor->SetProcessCallback(
      std::bind(&Synthetic::OnDisparityNormalizedProcess, this, _1, _2, _3));
  points_processor->SetProcessCallback(
      std::bind(&Synthetic::OnPointsProcess, this, _1, _2, _3));
  depth_processor->SetProcessCallback(
      std::bind(&Synthetic::OnDepthProcess, this, _1, _2, _3));

  rectify_processor->AddChild(disparity_processor);
  disparity_processor->AddChild(disparitynormalized_processor);
  disparity_processor->AddChild(points_processor);
  points_processor->AddChild(depth_processor);

  processor_ = rectify_processor;
}

bool Synthetic::OnRectifyProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(in)
  UNUSED(out)
  UNUSED(parent)
  return false;
}

bool Synthetic::OnDisparityProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(in)
  UNUSED(out)
  UNUSED(parent)
  return false;
}

bool Synthetic::OnDisparityNormalizedProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(in)
  UNUSED(out)
  UNUSED(parent)
  return false;
}

bool Synthetic::OnPointsProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(in)
  UNUSED(out)
  UNUSED(parent)
  return false;
}

bool Synthetic::OnDepthProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(in)
  UNUSED(out)
  UNUSED(parent)
  return false;
}

MYNTEYE_END_NAMESPACE
