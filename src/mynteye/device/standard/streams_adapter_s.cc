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

#include <iomanip>

#include "mynteye/logger.h"
#include "mynteye/device/types.h"

MYNTEYE_BEGIN_NAMESPACE
#define DEBUG_TIME_LIMIT false

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

  ImagePacket() = default;
  explicit ImagePacket(std::uint8_t *data) {
    from_data(data);
  }

  void from_data(std::uint8_t *data) {
    header = *data;
    size = *(data + 1);
    frame_id = (*(data + 2) << 8) | *(data + 3);
#if DEBUG_TIME_LIMIT
    uint32_t timestamp_test = (*(data + 4) << 24) | (*(data + 5) << 16) | (*(data + 6) << 8) | // NOLINT
                *(data + 7);
    timestamp_test += (std::uint32_t)4290000000;
    timestamp = timestamp_test;
#else
    timestamp = (*(data + 4) << 24) | (*(data + 5) << 16) | (*(data + 6) << 8) |
                *(data + 7);
#endif
    exposure_time = (*(data + 8) << 8) | *(data + 9);
    checksum = *(data + 10);
  }
};
#pragma pack(pop)

bool unpack_stereo_img_data(
    const void *data, const StreamRequest &request, ImgData *img) {
  CHECK_NOTNULL(img);

  auto data_new = reinterpret_cast<const std::uint8_t *>(data);
  std::size_t data_n =
      request.width * request.height * bytes_per_pixel(request.format);
  auto data_end = data_new + data_n;

  std::size_t packet_n = sizeof(ImagePacket) - 4;
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

  img->frame_id = img_packet.frame_id;
  // make timestamp unit from 10us to 1us
  img->timestamp = static_cast<uint64_t>(img_packet.timestamp) * 10;
  img->exposure_time = img_packet.exposure_time;
  return true;
}

// image pixels

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
