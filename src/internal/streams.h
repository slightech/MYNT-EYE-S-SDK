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
#ifndef MYNTEYE_INTERNAL_STREAMS_H_  // NOLINT
#define MYNTEYE_INTERNAL_STREAMS_H_
#pragma once

#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <vector>

#include "mynteye/callbacks.h"
#include "mynteye/mynteye.h"
#include "mynteye/types.h"

MYNTEYE_BEGIN_NAMESPACE

class Streams {
 public:
  using frame_t = device::Frame;
  using stream_data_t = device::StreamData;
  using stream_datas_t = std::vector<stream_data_t>;

  using unpack_img_data_t = std::function<bool(
      const void *data, const StreamRequest &request, ImgData *img)>;
  using unpack_img_pixels_t = std::function<bool(
      const void *data, const StreamRequest &request, frame_t *frame)>;

  explicit Streams(const std::vector<Stream> key_streams);
  ~Streams();

  void ConfigStream(
      const Capabilities &capability, const StreamRequest &request);

  bool PushStream(const Capabilities &capability, const void *data);

  void WaitForStreams();

  void ConfigStreamLimits(const Stream &stream, std::size_t max_data_size);
  std::size_t GetStreamDataMaxSize(const Stream &stream) const;

  stream_datas_t GetStreamDatas(const Stream &stream);
  stream_data_t GetLatestStreamData(const Stream &stream);

  const stream_datas_t &stream_datas(const Stream &stream);

 private:
  bool IsStreamCapability(const Capabilities &capability) const;
  bool HasStreamConfigRequest(const Capabilities &capability) const;

  const StreamRequest &GetStreamConfigRequest(
      const Capabilities &capability) const;

  bool HasStreamDatas(const Stream &stream) const;

  void AllocStreamData(const Stream &stream, const StreamRequest &request);
  void AllocStreamData(
      const Stream &stream, const StreamRequest &request, const Format &format);

  void DiscardStreamData(const Stream &stream);

  bool HasKeyStreamDatas() const;

  std::vector<Stream> key_streams_;

  std::vector<Capabilities> stream_capabilities_;
  std::map<Capabilities, StreamRequest> stream_config_requests_;

  std::map<Stream, unpack_img_data_t> unpack_img_data_map_;
  std::map<Stream, unpack_img_pixels_t> unpack_img_pixels_map_;

  std::map<Stream, std::size_t> stream_limits_map_;
  std::map<Stream, stream_datas_t> stream_datas_map_;

  std::mutex mtx_;
  std::condition_variable cv_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_STREAMS_H_ NOLINT
