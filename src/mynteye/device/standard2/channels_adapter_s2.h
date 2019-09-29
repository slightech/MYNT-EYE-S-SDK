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
#ifndef MYNTEYE_DEVICE_STANDARD2_CHANNELS_ADAPTER_S2_H_
#define MYNTEYE_DEVICE_STANDARD2_CHANNELS_ADAPTER_S2_H_
#pragma once

#include <cstdint>
#include <set>
#include <vector>

#include "mynteye/device/channel/channels.h"

MYNTEYE_BEGIN_NAMESPACE

class Standard2ChannelsAdapter : public ChannelsAdapter {
 public:
  explicit Standard2ChannelsAdapter(const Model &model);
  virtual ~Standard2ChannelsAdapter();

  std::int32_t GetAccelRangeDefault() override;
  std::vector<std::int32_t> GetAccelRangeValues() override;

  std::int32_t GetGyroRangeDefault() override;
  std::vector<std::int32_t> GetGyroRangeValues() override;

  void GetImuResPacket(const std::uint8_t *data, ImuResPacket *res) override;
  void GetImuResPacket2(
      const std::uint8_t *data,
      ImuResPacket2 *res,
      bool is_correspondence_on) override;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_STANDARD2_CHANNELS_ADAPTER_S2_H_
