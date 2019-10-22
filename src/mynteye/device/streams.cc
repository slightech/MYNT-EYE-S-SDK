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
#include "mynteye/device/streams.h"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <utility>

#include "mynteye/logger.h"
#include "mynteye/device/types.h"

MYNTEYE_BEGIN_NAMESPACE

Streams::Streams(const std::shared_ptr<StreamsAdapter> &adapter)
    : key_streams_(std::move(adapter->GetKeyStreams())),
      stream_capabilities_(std::move(adapter->GetStreamCapabilities())),
      unpack_img_data_map_(std::move(adapter->GetUnpackImgDataMap())),
      unpack_img_pixels_map_(std::move(adapter->GetUnpackImgPixelsMap())),
      timestamp_compensate_(0),
      is_nearly_before_timestamp_limmit_(0),
      current_datum_(0) {
  VLOG(2) << __func__;
}

Streams::~Streams() {
  VLOG(2) << __func__;
}

void Streams::ConfigStream(
    const Capabilities &capability, const StreamRequest &request) {
  if (!IsStreamCapability(capability)) {
    LOG(ERROR) << "Cannot config stream without stream capability";
    return;
  }
  VLOG(2) << "Config stream request of " << capability << ", " << request;
  stream_config_requests_[capability] = request;
}

void Streams::CheckTimeStampLimmit(std::shared_ptr<ImgData> img) {
    img->timestamp += (timestamp_compensate_ * 42949672960);
  // the timestamp nearly 8 imu frame before limmit
  if ((img->timestamp % 42949672960) > 42949606770 &&
      is_nearly_before_timestamp_limmit_ == 0) {
    current_datum_ = img->timestamp;
    timestamp_compensate_++;
    is_nearly_before_timestamp_limmit_ = LIMMIT_IMG_CHECK_DORMANCY_THRESHOLD;
  }
  if (is_nearly_before_timestamp_limmit_ > 0) {
    is_nearly_before_timestamp_limmit_--;
    uint64_t abs_tmp = 0;
    if (current_datum_ > img->timestamp) {
      abs_tmp = current_datum_ - img->timestamp;
    } else {
      abs_tmp = img->timestamp - current_datum_;
    }
    if (abs_tmp > (uint64_t)(42949672960/2)) {  // NOLINT
      img->timestamp -= 42949672960;
    }
  }
}

bool Streams::PushStream(const Capabilities &capability, const void *data) {
  if (!HasStreamConfigRequest(capability)) {
    LOG(FATAL) << "Cannot push stream without stream config request";
  }
  std::unique_lock<std::mutex> lock(mtx_);
  auto &&request = GetStreamConfigRequest(capability);
  bool pushed = false;
  switch (capability) {
    case Capabilities::STEREO:
    case Capabilities::STEREO_COLOR: {
      // alloc left
      AllocStreamData(capability, Stream::LEFT, request);
      auto &&left_data = stream_datas_map_[Stream::LEFT].back();
      // unpack img data
      if (unpack_img_data_map_[Stream::LEFT](
              data, request, left_data.img.get())) {
        left_data.frame_id = left_data.img->frame_id;
        // alloc right
        AllocStreamData(capability, Stream::RIGHT, request);
        auto &&right_data = stream_datas_map_[Stream::RIGHT].back();
        *right_data.img = *left_data.img;
        right_data.frame_id = left_data.img->frame_id;
        // unpack frame
        unpack_img_pixels_map_[Stream::LEFT](
            data, request, left_data.frame.get());
        unpack_img_pixels_map_[Stream::RIGHT](
            data, request, right_data.frame.get());
        pushed = true;
      } else {
        // discard left
        DiscardStreamData(Stream::LEFT);
        VLOG(2) << "Image packet is unaccepted, frame dropped";
        pushed = false;
      }
    } break;
    default:
      LOG(FATAL) << "Not supported " << capability << " now";
  }
  if (HasKeyStreamDatas())
    cv_.notify_one();
  return pushed;
}

bool Streams::PushStreamS1(const Capabilities &capability, const void *data) {
  if (!HasStreamConfigRequest(capability)) {
    LOG(FATAL) << "Cannot push stream without stream config request";
  }
  std::unique_lock<std::mutex> lock(mtx_);
  auto &&request = GetStreamConfigRequest(capability);
  bool pushed = false;
  switch (capability) {
    case Capabilities::STEREO:
    case Capabilities::STEREO_COLOR: {
      // alloc left
      AllocStreamData(capability, Stream::LEFT, request);
      auto &&left_data = stream_datas_map_[Stream::LEFT].back();
      // unpack img data
      if (unpack_img_data_map_[Stream::LEFT](
              data, request, left_data.img.get())) {
        left_data.frame_id = left_data.img->frame_id;
        CheckTimeStampLimmit(left_data.img);
        // alloc right
        AllocStreamData(capability, Stream::RIGHT, request);
        auto &&right_data = stream_datas_map_[Stream::RIGHT].back();
        *right_data.img = *left_data.img;
        right_data.frame_id = left_data.img->frame_id;
        // unpack frame
        unpack_img_pixels_map_[Stream::LEFT](
            data, request, left_data.frame.get());
        unpack_img_pixels_map_[Stream::RIGHT](
            data, request, right_data.frame.get());
        pushed = true;
      } else {
        // discard left
        DiscardStreamData(Stream::LEFT);
        VLOG(2) << "Image packet is unaccepted, frame dropped";
        pushed = false;
      }
    } break;
    default:
      LOG(FATAL) << "Not supported " << capability << " now";
  }
  if (HasKeyStreamDatas())
    cv_.notify_one();
  return pushed;
}

void Streams::WaitForStreams() {
  std::unique_lock<std::mutex> lock(mtx_);
  auto ready = std::bind(&Streams::HasKeyStreamDatas, this);
  if (!ready() && !cv_.wait_for(lock, std::chrono::seconds(2), ready)) {
    LOG(FATAL) << "Timeout waiting for key frames. Please use USB 3.0, and not "
                  "in virtual machine.";
  }
}

void Streams::ConfigStreamLimits(
    const Stream &stream, std::size_t max_data_size) {
  CHECK_GT(max_data_size, 0);
  stream_limits_map_[stream] = max_data_size;
}

std::size_t Streams::GetStreamDataMaxSize(const Stream &stream) const {
  try {
    return stream_limits_map_.at(stream);
  } catch (const std::out_of_range &e) {
    return 4;  // default stream data max size
  }
}

Streams::stream_datas_t Streams::GetStreamDatas(const Stream &stream) {
  std::unique_lock<std::mutex> lock(mtx_);
  if (!HasStreamDatas(stream)) {
    LOG(WARNING) << "There are no stream datas of " << stream
                 << ". Did you call WaitForStreams() before this?";
    return {};
  }
  auto datas = stream_datas_map_.at(stream);
  stream_datas_map_[stream].clear();
  return datas;
}

Streams::stream_data_t Streams::GetLatestStreamData(const Stream &stream) {
  std::unique_lock<std::mutex> lock(mtx_);
  if (!HasStreamDatas(stream)) {
    LOG(WARNING) << "There are no stream datas of " << stream
                 << ". Did you call WaitForStreams() before this?";
    return {};
  }
  auto data = stream_datas_map_.at(stream).back();
  stream_datas_map_[stream].clear();
  return data;
}

const Streams::stream_datas_t &Streams::stream_datas(const Stream &stream) {
  std::unique_lock<std::mutex> lock(mtx_);
  try {
    return stream_datas_map_.at(stream);
  } catch (const std::out_of_range &e) {
    // Add empty vector of this stream key
    stream_datas_map_[stream] = {};
    return stream_datas_map_.at(stream);
  }
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
  return stream_datas_map_.find(stream) != stream_datas_map_.end() &&
         !stream_datas_map_.at(stream).empty();
}

void Streams::AllocStreamData(const Capabilities &capability,
    const Stream &stream, const StreamRequest &request) {
  auto format = request.format;
  if (capability == Capabilities::STEREO) {
    format = Format::GREY;
  }
  AllocStreamData(capability, stream, request, format);
}

void Streams::AllocStreamData(const Capabilities &capability,
    const Stream &stream, const StreamRequest &request, const Format &format) {
  stream_data_t data;

  if (HasStreamDatas(stream)) {
    // If cached equal to limits_max, drop the oldest one.
    if (stream_datas_map_.at(stream).size() == GetStreamDataMaxSize(stream)) {
      auto &&datas = stream_datas_map_[stream];
      // reuse the dropped data
      data.img = datas.front().img;
      data.frame = datas.front().frame;
      data.frame_id = 0;
      datas.erase(datas.begin());
      VLOG(2) << "Stream data of " << stream << " is dropped as out of limits";
    }
  }

  if (stream == Stream::LEFT || stream == Stream::RIGHT) {
    if (!data.img) {
      data.img = std::make_shared<ImgData>();
    }
  } else {
    data.img = nullptr;
  }
  if (!data.frame) {
    auto width = request.width;
    if (capability == Capabilities::STEREO_COLOR) {
      width /= 2;  // split to half
    }
    data.frame =
      std::make_shared<frame_t>(width, request.height, format, nullptr);
  }
  data.frame_id = 0;
  stream_datas_map_[stream].push_back(data);
}

void Streams::DiscardStreamData(const Stream &stream) {
  // Must discard after alloc, otherwise at will out of range when no this key.
  if (stream_datas_map_.at(stream).size() > 0) {
    auto &&datas = stream_datas_map_[stream];
    datas.pop_back();
  } else {
    VLOG(2) << "Stream data of " << stream << " is empty, could not discard";
  }
}

bool Streams::HasKeyStreamDatas() const {
  for (auto &&s : key_streams_) {
    if (!HasStreamDatas(s))
      return false;
  }
  return true;
}

MYNTEYE_END_NAMESPACE
