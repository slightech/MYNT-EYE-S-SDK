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
#include "internal/streams.h"

#include <glog/logging.h>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <stdexcept>

#include "internal/types.h"

MYNTEYE_BEGIN_NAMESPACE

namespace {

bool unpack_stereo_img_data(
    const void *data, const StreamRequest &request, ImgData *img) {
  CHECK_NOTNULL(img);
  auto data_new = reinterpret_cast<const std::uint8_t *>(data);
  std::size_t data_n =
      request.width * request.height * bytes_per_pixel(request.format);
  auto data_end = data_new + data_n;

  std::size_t packet_n = sizeof(ImagePacket);
  std::vector<std::uint8_t> packet(packet_n);
  std::reverse_copy(data_end - packet_n, data_end, packet.begin());

  ImagePacket img_packet(packet.data());
  // LOG(INFO) << "ImagePacket: header=0x" << std::hex <<
  // static_cast<int>(img_packet.header)
  //   << ", size=0x" << std::hex << static_cast<int>(img_packet.size)
  //   << ", frame_id="<< std::dec << img_packet.frame_id
  //   << ", timestamp="<< std::dec << img_packet.timestamp
  //   << ", exposure_time="<< std::dec << img_packet.exposure_time
  //   << ", checksum=0x" << std::hex << static_cast<int>(img_packet.checksum);

  if (img_packet.header != 0x3B) {
    LOG(WARNING) << "Image packet header must be 0x3B, but 0x" << std::hex
                 << std::uppercase << std::setw(2) << std::setfill('0')
                 << static_cast<int>(img_packet.header) << " now";
    return false;
  }

  std::uint8_t checksum = 0;
  for (std::size_t i = 2, n = packet_n - 2; i <= n; i++) {  // content: [2,9]
    checksum = (checksum ^ packet[i]);
  }

  if (img_packet.checksum != checksum) {
    LOG(WARNING) << "Image packet checksum should be 0x" << std::hex
                 << std::uppercase << std::setw(2) << std::setfill('0')
                 << static_cast<int>(img_packet.checksum) << ", but 0x"
                 << std::setw(2) << std::setfill('0')
                 << static_cast<int>(checksum) << " now";
    return false;
  }

  img->frame_id = img_packet.frame_id;
  img->timestamp = img_packet.timestamp;
  img->exposure_time = img_packet.exposure_time;
  return true;
}

bool unpack_left_img_pixels(
    const void *data, const StreamRequest &request, Streams::frame_t *frame) {
  CHECK_NOTNULL(frame);
  CHECK_EQ(request.format, frame->format());
  auto data_new = reinterpret_cast<const std::uint8_t *>(data);
  if (request.format == Format::YUYV || request.format == Format::BGR888) {
    std::size_t n = request.format == Format::YUYV ? 2 : 3;
    std::size_t w = frame->width() * n;
    std::size_t h = frame->height();
    for (std::size_t i = 0; i < h; i++) {
      for (std::size_t j = 0; j < w; j++) {
        frame->data()[i * w + j] = *(data_new + 2 * i * w + j);
      }
    }
  } else if (request.format == Format::GREY) {
    std::size_t n = frame->width() * frame->height();
    for (std::size_t i = 0; i < n; i++) {
      frame->data()[i] = *(data_new + (i * 2));
    }
  } else {
    return false;
  }

  return true;
}

// TODO(Kalman): Too similar to 'unpack_left_img_pixels'
bool unpack_right_img_pixels(
    const void *data, const StreamRequest &request, Streams::frame_t *frame) {
  CHECK_NOTNULL(frame);
  CHECK_EQ(request.format, frame->format());
  auto data_new = reinterpret_cast<const std::uint8_t *>(data);
  if (request.format == Format::YUYV || request.format == Format::BGR888) {
    std::size_t n = request.format == Format::YUYV ? 2 : 3;
    std::size_t w = frame->width() * n;
    std::size_t h = frame->height();
    for (std::size_t i = 0; i < h; i++) {
      for (std::size_t j = 0; j < w; j++) {
        frame->data()[i * w + j] = *(data_new + (2 * i + 1) * w + j);
      }
    }
  } else if (request.format == Format::GREY) {
    std::size_t n = frame->width() * frame->height();
    for (std::size_t i = 0; i < n; i++) {
      frame->data()[i] = *(data_new + (i * 2 + 1));
    }
  } else {
    return false;
  }

  return true;
}

}  // namespace

Streams::Streams(const std::vector<Stream> key_streams)
    : key_streams_(key_streams),
      stream_capabilities_(
          {Capabilities::STEREO, Capabilities::COLOR, Capabilities::DEPTH,
           Capabilities::POINTS, Capabilities::FISHEYE, Capabilities::INFRARED,
           Capabilities::INFRARED2, Capabilities::STEREO_COLOR}),
      unpack_img_data_map_(
          {{Stream::LEFT, unpack_stereo_img_data},
           {Stream::RIGHT, unpack_stereo_img_data}}),
      unpack_img_pixels_map_(
          {{Stream::LEFT, unpack_left_img_pixels},
           {Stream::RIGHT, unpack_right_img_pixels}}) {
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

bool Streams::PushStream(const Capabilities &capability, const void *data) {
  if (!HasStreamConfigRequest(capability)) {
    LOG(FATAL) << "Cannot push stream without stream config request";
  }
  std::unique_lock<std::mutex> lock(mtx_);
  auto &&request = GetStreamConfigRequest(capability);
  bool pushed = false;
  switch (capability) {
    case Capabilities::STEREO_COLOR: {
      // alloc left
      AllocStreamData(Stream::LEFT, request);
      auto &&left_data = stream_datas_map_[Stream::LEFT].back();
      // unpack img data
      if (unpack_img_data_map_[Stream::LEFT](
              data, request, left_data.img.get())) {
        // alloc right
        AllocStreamData(Stream::RIGHT, request);
        auto &&right_data = stream_datas_map_[Stream::RIGHT].back();
        *right_data.img = *left_data.img;
        // unpack frame
        unpack_img_pixels_map_[Stream::LEFT](
            data, request, left_data.frame.get());
        unpack_img_pixels_map_[Stream::RIGHT](
            data, request, right_data.frame.get());
        pushed = true;
      } else {
        // discard left
        DiscardStreamData(Stream::LEFT);
        LOG(WARNING) << "Image packet is unaccepted, frame dropped";
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

void Streams::AllocStreamData(
    const Stream &stream, const StreamRequest &request) {
  AllocStreamData(stream, request, request.format);
}

void Streams::AllocStreamData(
    const Stream &stream, const StreamRequest &request, const Format &format) {
  stream_data_t data;

  if (HasStreamDatas(stream)) {
    // If cached equal to limits_max, drop the oldest one.
    if (stream_datas_map_.at(stream).size() == GetStreamDataMaxSize(stream)) {
      auto &&datas = stream_datas_map_[stream];
      // reuse the dropped data
      data.img = datas.front().img;
      data.frame = datas.front().frame;
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
    int width = request.width;
    if (format != Format::GREY)
      width /= 2;
    data.frame =
        std::make_shared<frame_t>(width, request.height, format, nullptr);
  }
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
