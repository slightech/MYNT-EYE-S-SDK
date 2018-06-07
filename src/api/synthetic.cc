// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "api/synthetic.h"

#include <glog/logging.h>

#include <algorithm>
#include <functional>
#include <stdexcept>

#include "api/plugin.h"
#include "api/processor/depth_processor.h"
#include "api/processor/disparity_normalized_processor.h"
#include "api/processor/disparity_processor.h"
#include "api/processor/object.h"
#include "api/processor/points_processor.h"
#include "api/processor/processor.h"
#include "api/processor/rectify_processor.h"
#include "device/device.h"

#define RECTIFY_PROC_PERIOD 0
#define DISPARITY_PROC_PERIOD 0
#define DISPARITY_NORM_PROC_PERIOD 0
#define POINTS_PROC_PERIOD 0
#define DEPTH_PROC_PERIOD 0

MYNTEYE_BEGIN_NAMESPACE

namespace {

cv::Mat frame2mat(const std::shared_ptr<device::Frame> &frame) {
  // TODO(JohnZhao) Support different format frame to cv::Mat
  CHECK_EQ(frame->format(), Format::GREY);
  return cv::Mat(frame->height(), frame->width(), CV_8UC1, frame->data());
}

api::StreamData data2api(const device::StreamData &data) {
  return {data.img, frame2mat(data.frame), data.frame};
}

}  // namespace

Synthetic::Synthetic(API *api) : api_(api), plugin_(nullptr) {
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
          stream,
          [this, stream](const device::StreamData &data) {
            auto &&stream_data = data2api(data);
            ProcessNativeStream(stream, stream_data);
            // Need mutex if set callback after start
            if (HasStreamCallback(stream)) {
              stream_callbacks_.at(stream)(stream_data);
            }
          },
          true);
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
    switch (stream) {
      case Stream::LEFT_RECTIFIED: {
        auto &&processor = find_processor<RectifyProcessor>(processor_);
        Object *out = processor->GetOutput();
        if (out != nullptr) {
          ObjMat2 *output = Object::Cast<ObjMat2>(out);
          return {nullptr, output->first, nullptr};
        }
        VLOG(2) << "Rectify not ready now";
      } break;
      case Stream::RIGHT_RECTIFIED: {
        auto &&processor = find_processor<RectifyProcessor>(processor_);
        Object *out = processor->GetOutput();
        if (out != nullptr) {
          ObjMat2 *output = Object::Cast<ObjMat2>(out);
          return {nullptr, output->second, nullptr};
        }
        VLOG(2) << "Rectify not ready now";
      } break;
      case Stream::DISPARITY: {
        auto &&processor = find_processor<DisparityProcessor>(processor_);
        Object *out = processor->GetOutput();
        if (out != nullptr) {
          ObjMat *output = Object::Cast<ObjMat>(out);
          return {nullptr, output->value, nullptr};
        }
        VLOG(2) << "Disparity not ready now";
      } break;
      case Stream::DISPARITY_NORMALIZED: {
        auto &&processor =
            find_processor<DisparityNormalizedProcessor>(processor_);
        Object *out = processor->GetOutput();
        if (out != nullptr) {
          ObjMat *output = Object::Cast<ObjMat>(out);
          return {nullptr, output->value, nullptr};
        }
        VLOG(2) << "Disparity normalized not ready now";
      } break;
      case Stream::POINTS: {
        auto &&processor = find_processor<PointsProcessor>(processor_);
        Object *out = processor->GetOutput();
        if (out != nullptr) {
          ObjMat *output = Object::Cast<ObjMat>(out);
          return {nullptr, output->value, nullptr};
        }
        VLOG(2) << "Points not ready now";
      } break;
      case Stream::DEPTH: {
        auto &&processor = find_processor<DepthProcessor>(processor_);
        Object *out = processor->GetOutput();
        if (out != nullptr) {
          ObjMat *output = Object::Cast<ObjMat>(out);
          return {nullptr, output->value, nullptr};
        }
        VLOG(2) << "Depth not ready now";
      } break;
      default:
        break;
    }
    return {};  // frame.empty() == true
  } else {
    LOG(ERROR) << "Failed to get stream data of " << stream
               << ", unsupported or disabled";
    return {};  // frame.empty() == true
  }
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
    return {GetStreamData(stream)};
  } else {
    LOG(ERROR) << "Failed to get stream data of " << stream
               << ", unsupported or disabled";
  }
  return {};
}

void Synthetic::SetPlugin(std::shared_ptr<Plugin> plugin) {
  plugin_ = plugin;
}

bool Synthetic::HasPlugin() const {
  return plugin_ != nullptr;
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
  auto &&rectify_processor =
      std::make_shared<RectifyProcessor>(api_->device(), RECTIFY_PROC_PERIOD);
  auto &&disparity_processor =
      std::make_shared<DisparityProcessor>(DISPARITY_PROC_PERIOD);
  auto &&disparitynormalized_processor =
      std::make_shared<DisparityNormalizedProcessor>(
          DISPARITY_NORM_PROC_PERIOD);
  auto &&points_processor = std::make_shared<PointsProcessor>(
      rectify_processor->Q, POINTS_PROC_PERIOD);
  auto &&depth_processor = std::make_shared<DepthProcessor>(DEPTH_PROC_PERIOD);

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

  rectify_processor->SetPostProcessCallback(
      std::bind(&Synthetic::OnRectifyPostProcess, this, _1));
  disparity_processor->SetPostProcessCallback(
      std::bind(&Synthetic::OnDisparityPostProcess, this, _1));
  disparitynormalized_processor->SetPostProcessCallback(
      std::bind(&Synthetic::OnDisparityNormalizedPostProcess, this, _1));
  points_processor->SetPostProcessCallback(
      std::bind(&Synthetic::OnPointsPostProcess, this, _1));
  depth_processor->SetPostProcessCallback(
      std::bind(&Synthetic::OnDepthPostProcess, this, _1));

  rectify_processor->AddChild(disparity_processor);
  disparity_processor->AddChild(disparitynormalized_processor);
  disparity_processor->AddChild(points_processor);
  points_processor->AddChild(depth_processor);

  processor_ = rectify_processor;
}

void Synthetic::ProcessNativeStream(
    const Stream &stream, const api::StreamData &data) {
  bool done = false;
  static api::StreamData left_data, right_data;
  if (stream == Stream::LEFT) {
    left_data = data;
    done = true;
  } else if (stream == Stream::RIGHT) {
    right_data = data;
    done = true;
  }
  if (done && left_data.img && right_data.img &&
      left_data.img->frame_id == right_data.img->frame_id) {
    auto &&processor = find_processor<RectifyProcessor>(processor_);
    processor->Process(ObjMat2{left_data.frame, right_data.frame});
  }
  if (done)
    return;

  auto &&process_childs = [this, &stream](
      const std::string &name, const Object &obj) {
    auto &&processor = find_processor<Processor>(processor_, name);
    for (auto child : processor->GetChilds()) {
      child->Process(obj);
    }
  };

  static api::StreamData left_rect_data, right_rect_data;
  if (stream == Stream::LEFT_RECTIFIED) {
    left_rect_data = data;
    done = true;
  } else if (stream == Stream::RIGHT_RECTIFIED) {
    right_rect_data = data;
    done = true;
  }
  if (done && left_rect_data.img && right_rect_data.img &&
      left_rect_data.img->frame_id == right_rect_data.img->frame_id) {
    process_childs(
        RectifyProcessor::NAME,
        ObjMat2{left_rect_data.frame, right_rect_data.frame});
  }
  if (done)
    return;

  switch (stream) {
    case Stream::DISPARITY: {
      process_childs(DisparityProcessor::NAME, ObjMat{data.frame});
    } break;
    case Stream::DISPARITY_NORMALIZED: {
      process_childs(DisparityNormalizedProcessor::NAME, ObjMat{data.frame});
    } break;
    case Stream::POINTS: {
      process_childs(PointsProcessor::NAME, ObjMat{data.frame});
    } break;
    case Stream::DEPTH: {
      process_childs(DepthProcessor::NAME, ObjMat{data.frame});
    } break;
    default:
      break;
  }
}

bool Synthetic::OnRectifyProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(parent)
  if (plugin_ && plugin_->OnRectifyProcess(in, out)) {
    return true;
  }
  return GetStreamEnabledMode(Stream::LEFT_RECTIFIED) != MODE_SYNTHETIC;
  // && GetStreamEnabledMode(Stream::RIGHT_RECTIFIED) != MODE_SYNTHETIC
}

bool Synthetic::OnDisparityProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(parent)
  if (plugin_ && plugin_->OnDisparityProcess(in, out)) {
    return true;
  }
  return GetStreamEnabledMode(Stream::DISPARITY) != MODE_SYNTHETIC;
}

bool Synthetic::OnDisparityNormalizedProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(parent)
  if (plugin_ && plugin_->OnDisparityNormalizedProcess(in, out)) {
    return true;
  }
  return GetStreamEnabledMode(Stream::DISPARITY_NORMALIZED) != MODE_SYNTHETIC;
}

bool Synthetic::OnPointsProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(parent)
  if (plugin_ && plugin_->OnPointsProcess(in, out)) {
    return true;
  }
  return GetStreamEnabledMode(Stream::POINTS) != MODE_SYNTHETIC;
}

bool Synthetic::OnDepthProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(parent)
  if (plugin_ && plugin_->OnDepthProcess(in, out)) {
    return true;
  }
  return GetStreamEnabledMode(Stream::DEPTH) != MODE_SYNTHETIC;
}

void Synthetic::OnRectifyPostProcess(Object *const out) {
  const ObjMat2 *output = Object::Cast<ObjMat2>(out);
  if (HasStreamCallback(Stream::LEFT_RECTIFIED)) {
    stream_callbacks_.at(Stream::LEFT_RECTIFIED)(
        {nullptr, output->first, nullptr});
  }
  if (HasStreamCallback(Stream::RIGHT_RECTIFIED)) {
    stream_callbacks_.at(Stream::RIGHT_RECTIFIED)(
        {nullptr, output->second, nullptr});
  }
}

void Synthetic::OnDisparityPostProcess(Object *const out) {
  const ObjMat *output = Object::Cast<ObjMat>(out);
  if (HasStreamCallback(Stream::DISPARITY)) {
    stream_callbacks_.at(Stream::DISPARITY)({nullptr, output->value, nullptr});
  }
}

void Synthetic::OnDisparityNormalizedPostProcess(Object *const out) {
  const ObjMat *output = Object::Cast<ObjMat>(out);
  if (HasStreamCallback(Stream::DISPARITY_NORMALIZED)) {
    stream_callbacks_.at(Stream::DISPARITY_NORMALIZED)(
        {nullptr, output->value, nullptr});
  }
}

void Synthetic::OnPointsPostProcess(Object *const out) {
  const ObjMat *output = Object::Cast<ObjMat>(out);
  if (HasStreamCallback(Stream::POINTS)) {
    stream_callbacks_.at(Stream::POINTS)({nullptr, output->value, nullptr});
  }
}

void Synthetic::OnDepthPostProcess(Object *const out) {
  const ObjMat *output = Object::Cast<ObjMat>(out);
  if (HasStreamCallback(Stream::DEPTH)) {
    stream_callbacks_.at(Stream::DEPTH)({nullptr, output->value, nullptr});
  }
}

MYNTEYE_END_NAMESPACE
