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

namespace {

cv::Mat frame2mat(const std::shared_ptr<device::Frame> &frame) {
  // TODO(JohnZhao) Support different format frame to cv::Mat
  CHECK_EQ(frame->format(), Format::GREY);
  return cv::Mat(frame->height(), frame->width(), CV_8UC1, frame->data());
}

api::StreamData data2api(const device::StreamData &data) {
  return {data.img, frame2mat(data.frame)};
}

}  // namespace

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

bool Synthetic::Supports(const Stream &stream) const {
  return stream_supports_mode_.find(stream) != stream_supports_mode_.end();
}

Synthetic::mode_t Synthetic::SupportsMode(const Stream &stream) const {
  try {
    return stream_supports_mode_.at(stream);
  } catch (const std::out_of_range &e) {
    return MODE_LAST;
  }
}

void Synthetic::EnableStreamData(const Stream &stream) {
  EnableStreamData(stream, 0);
}

void Synthetic::DisableStreamData(const Stream &stream) {
  DisableStreamData(stream, 0);
}

bool Synthetic::IsStreamDataEnabled(const Stream &stream) const {
  return stream_enabled_mode_.find(stream) != stream_enabled_mode_.end();
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

void Synthetic::StartVideoStreaming() {
  auto &&device = api_->device();
  for (auto &&it = stream_supports_mode_.begin();
       it != stream_supports_mode_.end(); it++) {
    if (it->second == MODE_NATIVE) {
      auto &&stream = it->first;
      device->SetStreamCallback(
          stream, [this, stream](const device::StreamData &data) {
            auto &&stream_data = data2api(data);
            Process(stream, stream_data);
            // Need mutex if set callback after start
            if (HasStreamCallback(stream)) {
              stream_callbacks_.at(stream)(stream_data);
            }
          });
    }
  }
  device->Start(Source::VIDEO_STREAMING);
}

void Synthetic::StopVideoStreaming() {
  auto &&device = api_->device();
  for (auto &&it = stream_supports_mode_.begin();
       it != stream_supports_mode_.end(); it++) {
    if (it->second == MODE_NATIVE) {
      device->SetStreamCallback(it->first, nullptr);
    }
  }
  device->Stop(Source::VIDEO_STREAMING);
}

void Synthetic::WaitForStreams() {
  api_->device()->WaitForStreams();
}

api::StreamData Synthetic::GetStreamData(const Stream &stream) {
  auto &&mode = GetStreamEnabledMode(stream);
  if (mode == MODE_NATIVE) {
    auto &&device = api_->device();
    return data2api(device->GetLatestStreamData(stream));
  } else if (mode == MODE_SYNTHETIC) {
    // TODO(JohnZhao)
  } else {
    LOG(ERROR) << "Failed to get stream data of " << stream
               << ", unsupported or disabled";
  }
  return {};
}

std::vector<api::StreamData> Synthetic::GetStreamDatas(const Stream &stream) {
  auto &&mode = GetStreamEnabledMode(stream);
  if (mode == MODE_NATIVE) {
    auto &&device = api_->device();
    std::vector<api::StreamData> datas;
    for (auto &&data : device->GetStreamDatas(stream)) {
      datas.push_back(data2api(data));
    }
    return datas;
  } else if (mode == MODE_SYNTHETIC) {
    // TODO(JohnZhao)
  } else {
    LOG(ERROR) << "Failed to get stream data of " << stream
               << ", unsupported or disabled";
  }
  return {};
}

void Synthetic::InitStreamSupports() {
  auto &&device = api_->device();
  if (device->Supports(Stream::LEFT) && device->Supports(Stream::RIGHT)) {
    stream_supports_mode_[Stream::LEFT] = MODE_NATIVE;
    stream_supports_mode_[Stream::RIGHT] = MODE_NATIVE;

    std::vector<Stream> stream_chain{
        Stream::LEFT_RECTIFIED, Stream::RIGHT_RECTIFIED,
        Stream::DISPARITY,      Stream::DISPARITY_NORMALIZED,
        Stream::POINTS,         Stream::DEPTH};
    for (auto &&stream : stream_chain) {
      if (device->Supports(stream)) {
        stream_supports_mode_[stream] = MODE_NATIVE;
      } else {
        stream_supports_mode_[stream] = MODE_SYNTHETIC;
      }
    }
  }

  // Enabled native streams by default
  for (auto &&it = stream_supports_mode_.begin();
       it != stream_supports_mode_.end(); it++) {
    if (it->second == MODE_NATIVE) {
      stream_enabled_mode_[it->first] = MODE_NATIVE;
    }
  }
}

Synthetic::mode_t Synthetic::GetStreamEnabledMode(const Stream &stream) const {
  try {
    return stream_enabled_mode_.at(stream);
  } catch (const std::out_of_range &e) {
    return MODE_LAST;
  }
}

bool Synthetic::IsStreamEnabledNative(const Stream &stream) const {
  return GetStreamEnabledMode(stream) == MODE_NATIVE;
}

bool Synthetic::IsStreamEnabledSynthetic(const Stream &stream) const {
  return GetStreamEnabledMode(stream) == MODE_SYNTHETIC;
}

void Synthetic::EnableStreamData(const Stream &stream, std::uint32_t depth) {
  if (IsStreamDataEnabled(stream))
    return;
  // Activate processors of synthetic stream
  switch (stream) {
    case Stream::LEFT_RECTIFIED: {
      if (!IsStreamDataEnabled(Stream::LEFT))
        break;
      stream_enabled_mode_[stream] = MODE_SYNTHETIC;
      CHECK(ActivateProcessor<RectifyProcessor>());
    }
      return;
    case Stream::RIGHT_RECTIFIED: {
      if (!IsStreamDataEnabled(Stream::RIGHT))
        break;
      stream_enabled_mode_[stream] = MODE_SYNTHETIC;
      CHECK(ActivateProcessor<RectifyProcessor>());
    }
      return;
    case Stream::DISPARITY: {
      stream_enabled_mode_[stream] = MODE_SYNTHETIC;
      EnableStreamData(Stream::LEFT_RECTIFIED, depth + 1);
      EnableStreamData(Stream::RIGHT_RECTIFIED, depth + 1);
      CHECK(ActivateProcessor<DisparityProcessor>());
    }
      return;
    case Stream::DISPARITY_NORMALIZED: {
      stream_enabled_mode_[stream] = MODE_SYNTHETIC;
      EnableStreamData(Stream::DISPARITY, depth + 1);
      CHECK(ActivateProcessor<DisparityNormalizedProcessor>());
    }
      return;
    case Stream::POINTS: {
      stream_enabled_mode_[stream] = MODE_SYNTHETIC;
      EnableStreamData(Stream::DISPARITY, depth + 1);
      CHECK(ActivateProcessor<PointsProcessor>());
    }
      return;
    case Stream::DEPTH: {
      stream_enabled_mode_[stream] = MODE_SYNTHETIC;
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
  if (stream_enabled_mode_[stream] != MODE_NATIVE) {
    stream_enabled_mode_.erase(stream);
    switch (stream) {
      case Stream::LEFT_RECTIFIED: {
        if (IsStreamEnabledSynthetic(Stream::RIGHT_RECTIFIED)) {
          DisableStreamData(Stream::RIGHT_RECTIFIED, depth + 1);
        }
        if (IsStreamEnabledSynthetic(Stream::DISPARITY)) {
          DisableStreamData(Stream::DISPARITY, depth + 1);
        }
        DeactivateProcessor<RectifyProcessor>();
      } break;
      case Stream::RIGHT_RECTIFIED: {
        if (IsStreamEnabledSynthetic(Stream::LEFT_RECTIFIED)) {
          DisableStreamData(Stream::LEFT_RECTIFIED, depth + 1);
        }
        if (IsStreamEnabledSynthetic(Stream::DISPARITY)) {
          DisableStreamData(Stream::DISPARITY, depth + 1);
        }
        DeactivateProcessor<RectifyProcessor>();
      } break;
      case Stream::DISPARITY: {
        if (IsStreamEnabledSynthetic(Stream::DISPARITY_NORMALIZED)) {
          DisableStreamData(Stream::DISPARITY_NORMALIZED, depth + 1);
        }
        if (IsStreamEnabledSynthetic(Stream::POINTS)) {
          DisableStreamData(Stream::POINTS, depth + 1);
        }
        DeactivateProcessor<DisparityProcessor>();
      } break;
      case Stream::DISPARITY_NORMALIZED: {
        DeactivateProcessor<DisparityNormalizedProcessor>();
      } break;
      case Stream::POINTS: {
        if (IsStreamEnabledSynthetic(Stream::DEPTH)) {
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

void Synthetic::Process(const Stream &stream, const api::StreamData &data) {
  static api::StreamData left_data, right_data;
  if (stream == Stream::LEFT) {
    left_data = data;
  } else if (stream == Stream::RIGHT) {
    right_data = data;
  }
  if (left_data.img && right_data.img &&
      left_data.img->frame_id == right_data.img->frame_id) {
    // TODO(JohnZhao)
  }
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
