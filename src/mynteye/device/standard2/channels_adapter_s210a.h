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
#ifndef MYNTEYE_DEVICE_STANDARD2_CHANNELS_ADAPTER_S210A_H_
#define MYNTEYE_DEVICE_STANDARD2_CHANNELS_ADAPTER_S210A_H_
#pragma once

#include <cstdint>
#include <set>
#include <vector>

#include "mynteye/device/channels/channels.h"

MYNTEYE_BEGIN_NAMESPACE

class Standard210aChannelsAdapter : public ChannelsAdapter {
 public:
  Standard210aChannelsAdapter();
  virtual ~Standard210aChannelsAdapter();

  std::set<Option> GetOptionSupports() override;

  std::int32_t GetAccelRangeDefault() override;
  std::vector<std::int32_t> GetAccelRangeValues() override;

  std::int32_t GetGyroRangeDefault() override;
  std::vector<std::int32_t> GetGyroRangeValues() override;

  void GetImuResPacket(const std::uint8_t *data, ImuResPacket *res) override;

  std::size_t GetImgParamsFromData(
      const std::uint8_t *data, const Version *version,
      Channels::img_params_t *img_params) override;
  std::size_t SetImgParamsToData(
      const Channels::img_params_t *img_params, const Version *version,
      std::uint8_t *data) override;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_STANDARD2_CHANNELS_ADAPTER_S210A_H_
