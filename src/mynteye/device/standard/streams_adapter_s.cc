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
#include "mynteye/device/standard/streams_adapter_s.h"

#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

namespace {

bool unpack_left_img_pixels(
    const void *data, const StreamRequest &request, Streams::frame_t *frame) {
  CHECK_NOTNULL(frame);
  CHECK_EQ(request.format, Format::YUYV);
  CHECK_EQ(frame->format(), Format::GREY);
  auto data_new = reinterpret_cast<const std::uint8_t *>(data);
  std::size_t n = frame->width() * frame->height();
  for (std::size_t i = 0; i < n; i++) {
    frame->data()[i] = *(data_new + (i * 2));
  }
  return true;
}

bool unpack_right_img_pixels(
    const void *data, const StreamRequest &request, Streams::frame_t *frame) {
  CHECK_NOTNULL(frame);
  CHECK_EQ(request.format, Format::YUYV);
  CHECK_EQ(frame->format(), Format::GREY);
  auto data_new = reinterpret_cast<const std::uint8_t *>(data);
  std::size_t n = frame->width() * frame->height();
  for (std::size_t i = 0; i < n; i++) {
    frame->data()[i] = *(data_new + (i * 2 + 1));
  }
  return true;
}

}  // namespace

StandardStreamsAdapter::StandardStreamsAdapter() {
}

StandardStreamsAdapter::~StandardStreamsAdapter() {
}

std::vector<Stream> StandardStreamsAdapter::GetKeyStreams() {
  return {Stream::LEFT, Stream::RIGHT};
}

std::vector<Capabilities> StandardStreamsAdapter::GetStreamCapabilities() {
  return {Capabilities::STEREO};
}

std::map<Stream, Streams::unpack_img_data_t>
StandardStreamsAdapter::GetUnpackImgDataMap() {
  return {
    {Stream::LEFT, unpack_stereo_img_data},
    {Stream::RIGHT, unpack_stereo_img_data}
  };
}

std::map<Stream, Streams::unpack_img_pixels_t>
StandardStreamsAdapter::GetUnpackImgPixelsMap() {
  return {
    {Stream::LEFT, unpack_left_img_pixels},
    {Stream::RIGHT, unpack_right_img_pixels}
  };
}

MYNTEYE_END_NAMESPACE
