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

#include <iomanip>

#include "mynteye/logger.h"
#include "mynteye/device/types.h"

#define DEBUG_TIME_LIMIT false

MYNTEYE_BEGIN_NAMESPACE

namespace {

// image info

#pragma pack(push, 1)
struct ImagePacket {
  std::uint8_t header;
  std::uint8_t size;
  std::uint16_t frame_id;
  std::uint64_t timestamp;
  std::uint16_t exposure_time;
  std::uint8_t checksum;
  // Is external time source
  bool is_ets;
  ImagePacket() = default;
  explicit ImagePacket(std::uint8_t *data) {
    from_data(data);
  }

  void from_data(std::uint8_t *data) {
    std::uint32_t timestamp_l;
    std::uint32_t timestamp_h;
    header = *data & 0x7F;
    is_ets = ((*data & 0x80) == 0x80);
    size = *(data + 1);
    frame_id = (*(data + 2) << 8) | *(data + 3);
    timestamp_h = (*(data + 4) << 24) | (*(data + 5) << 16) |
                  (*(data + 6) << 8) | *(data + 7);
    timestamp_l = (*(data + 8) << 24) | (*(data + 9) << 16) |
                  (*(data + 10) << 8) | *(data + 11);
    timestamp = (static_cast<std::uint64_t>(timestamp_h) << 32) | timestamp_l;
#if DEBUG_TIME_LIMIT
    timestamp += 4200000000;
#endif
    exposure_time = (*(data + 12) << 8) | *(data + 13);
    checksum = *(data + 14);
  }
};
#pragma pack(pop)

// image pixels

bool unpack_left_img_pixels(
    const void *data, const StreamRequest &request, Streams::frame_t *frame) {
  CHECK_NOTNULL(frame);
  CHECK_EQ(request.format, Format::YUYV);
  CHECK_EQ(frame->format(), Format::YUYV);
  auto data_new = reinterpret_cast<const std::uint8_t *>(data);
  std::size_t n = 2;
  std::size_t w = frame->width() * n;
  std::size_t h = frame->height();
  for (std::size_t i = 0; i < h; i++) {
    for (std::size_t j = 0; j < w; j++) {
      frame->data()[i * w + j] = *(data_new + 2 * i * w + j);
    }
  }
  return true;
}

bool unpack_right_img_pixels(
    const void *data, const StreamRequest &request, Streams::frame_t *frame) {
  CHECK_NOTNULL(frame);
  CHECK_EQ(request.format, Format::YUYV);
  CHECK_EQ(frame->format(), Format::YUYV);
  auto data_new = reinterpret_cast<const std::uint8_t *>(data);
  std::size_t n = 2;
  std::size_t w = frame->width() * n;
  std::size_t h = frame->height();
  for (std::size_t i = 0; i < h; i++) {
    for (std::size_t j = 0; j < w; j++) {
      frame->data()[i * w + j] = *(data_new + (2 * i + 1) * w + j);
    }
  }
  return true;
}

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
    VLOG(2) << "Image packet header must be 0x3B, but 0x" << std::hex
            << std::uppercase << std::setw(2) << std::setfill('0')
            << static_cast<int>(img_packet.header) << " now";
    return false;
  }
/*
  std::uint8_t checksum = 0;
  for (std::size_t i = 2, n = packet_n - 2; i <= n; i++) {  // content: [2,9]
    checksum = (checksum ^ packet[i]);
  }

  if (img_packet.checksum != checksum) {
    VLOG(2) << "Image packet checksum should be 0x" << std::hex
            << std::uppercase << std::setw(2) << std::setfill('0')
            << static_cast<int>(img_packet.checksum) << ", but 0x"
            << std::setw(2) << std::setfill('0') << static_cast<int>(checksum)
            << " now";
    return false;
  }
*/
  img->frame_id = img_packet.frame_id;
  img->timestamp = img_packet.timestamp;
  img->exposure_time = img_packet.exposure_time;
  img->is_ets = img_packet.is_ets;
  return true;
}

}  // namespace

namespace s210a {

// image pixels

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

}  // namespace s210a

Standard2StreamsAdapter::Standard2StreamsAdapter(const Model &model)
  : model_(model) {
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
  switch (model_) {
    case Model::STANDARD210A:
      return {
        {Stream::LEFT, s210a::unpack_left_img_pixels},
        {Stream::RIGHT, s210a::unpack_right_img_pixels}
      };
    case Model::STANDARD2:
    case Model::STANDARD200B:
    default:
      return {
        {Stream::LEFT, unpack_left_img_pixels},
        {Stream::RIGHT, unpack_right_img_pixels}
      };
  }
}

MYNTEYE_END_NAMESPACE
