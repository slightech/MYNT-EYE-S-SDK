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
#ifndef MYNTEYE_DEVICE_STANDARD2_STREAMS_ADAPTER_S2_H_
#define MYNTEYE_DEVICE_STANDARD2_STREAMS_ADAPTER_S2_H_
#pragma once

#include <map>
#include <memory>
#include <vector>

#include "mynteye/device/streams.h"

MYNTEYE_BEGIN_NAMESPACE

class Standard2StreamsAdapter : public StreamsAdapter {
 public:
  explicit Standard2StreamsAdapter(const Model &model);
  virtual ~Standard2StreamsAdapter();

  std::vector<Stream> GetKeyStreams() override;
  std::vector<Capabilities> GetStreamCapabilities() override;

  std::map<Stream, Streams::unpack_img_data_t>
  GetUnpackImgDataMap() override;
  std::map<Stream, Streams::unpack_img_pixels_t>
  GetUnpackImgPixelsMap() override;

 private:
  Model model_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_STANDARD2_STREAMS_ADAPTER_S2_H_
