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
#include "mynteye/device/standard2/streams_adapter_s2.h"

#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

namespace {

bool unpack_left_img_pixels(
    const void *data, const StreamRequest &request, Streams::frame_t *frame) {
  CHECK_NOTNULL(frame);
  CHECK_EQ(request.format, Format::BGR888);
  CHECK_EQ(frame->format(), Format::BGR888);
  auto data_new = reinterpret_cast<const std::uint8_t *>(data);
  std::size_t n = 3;
  std::size_t w = frame->width();
  std::size_t h = frame->height();
  for (std::size_t i = 0; i < h; i++) {
    for (std::size_t j = 0; j < w; j++) {
      frame->data()[(i * w + j) * n] =
        *(data_new + (2 * i * w + j) * n + 2);
      frame->data()[(i * w + j) * n + 1] =
        *(data_new + (2 * i * w + j) * n + 1);
      frame->data()[(i * w + j) * n + 2] =
        *(data_new + (2 * i * w + j) * n);
    }
  }
  return true;
}

bool unpack_right_img_pixels(
    const void *data, const StreamRequest &request, Streams::frame_t *frame) {
  CHECK_NOTNULL(frame);
  CHECK_EQ(request.format, Format::BGR888);
  CHECK_EQ(frame->format(), Format::BGR888);
  auto data_new = reinterpret_cast<const std::uint8_t *>(data);
  std::size_t n = 3;
  std::size_t w = frame->width();
  std::size_t h = frame->height();
  for (std::size_t i = 0; i < h; i++) {
    for (std::size_t j = 0; j < w; j++) {
      frame->data()[(i * w + j) * n] =
        *(data_new + ((2 * i + 1) * w + j) * n + 2);
      frame->data()[(i * w + j) * n + 1] =
        *(data_new + ((2 * i + 1) * w + j) * n + 1);
      frame->data()[(i * w + j) * n + 2] =
        *(data_new + ((2 * i + 1) * w + j) * n);
    }
  }
  return true;
}

}  // namespace

Standard2StreamsAdapter::Standard2StreamsAdapter() {
}

Standard2StreamsAdapter::~Standard2StreamsAdapter() {
}

std::vector<Stream> Standard2StreamsAdapter::GetKeyStreams() {
  return {Stream::LEFT, Stream::RIGHT};
}

std::vector<Capabilities> Standard2StreamsAdapter::GetStreamCapabilities() {
  return {Capabilities::STEREO_COLOR};
}

std::map<Stream, Streams::unpack_img_data_t>
Standard2StreamsAdapter::GetUnpackImgDataMap() {
  return {
    {Stream::LEFT, unpack_stereo_img_data},
    {Stream::RIGHT, unpack_stereo_img_data}
  };
}

std::map<Stream, Streams::unpack_img_pixels_t>
Standard2StreamsAdapter::GetUnpackImgPixelsMap() {
  return {
    {Stream::LEFT, unpack_left_img_pixels},
    {Stream::RIGHT, unpack_right_img_pixels}
  };
}

MYNTEYE_END_NAMESPACE
