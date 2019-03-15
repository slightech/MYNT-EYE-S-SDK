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
#ifndef MYNTEYE_DEVICE_CHANNEL_BYTES_H_
#define MYNTEYE_DEVICE_CHANNEL_BYTES_H_
#pragma once

#include <algorithm>
#include <string>

#include "mynteye/mynteye.h"
#include "mynteye/types.h"
#include "mynteye/device/channel/def.h"
#include "mynteye/device/types.h"

MYNTEYE_BEGIN_NAMESPACE

namespace bytes {

// from

template <typename T>
T _from_data(const std::uint8_t *data) {
  std::size_t size = sizeof(T) / sizeof(std::uint8_t);
  T value = 0;
  for (std::size_t i = 0; i < size; i++) {
    value |= data[i] << (8 * (size - i - 1));
  }
  return value;
}

template <>
inline double _from_data(const std::uint8_t *data) {
  return *(reinterpret_cast<const double *>(data));
}

std::string _from_data(const std::uint8_t *data, std::size_t count);

// from types

std::size_t from_data(IntrinsicsBase *in, const std::uint8_t *data,
    bool get_size);
std::size_t from_data(IntrinsicsPinhole *in, const std::uint8_t *data,
    bool get_size);
std::size_t from_data(IntrinsicsEquidistant *in, const std::uint8_t *data,
    bool get_size);

std::size_t from_data(ImuIntrinsics *in, const std::uint8_t *data,
    bool get_size);

std::size_t from_data(Extrinsics *ex, const std::uint8_t *data);

// to

template <typename T>
std::size_t _to_data(T value, std::uint8_t *data) {
  std::size_t size = sizeof(T) / sizeof(std::uint8_t);
  for (std::size_t i = 0; i < size; i++) {
    data[i] = static_cast<std::uint8_t>((value >> (8 * (size - i - 1))) & 0xFF);
  }
  return size;
}

template <>
inline std::size_t _to_data(double value, std::uint8_t *data) {
  std::uint8_t *val = reinterpret_cast<std::uint8_t *>(&value);
  std::copy(val, val + 8, data);
  return 8;
}

std::size_t _to_data(std::string value, std::uint8_t *data, std::size_t count);

// to types

std::size_t to_data(const IntrinsicsBase *in, std::uint8_t *data,
    bool set_size);
std::size_t to_data(const IntrinsicsPinhole *in, std::uint8_t *data,
    bool set_size);
std::size_t to_data(const IntrinsicsEquidistant *in, std::uint8_t *data,
    bool set_size);

std::size_t to_data(const ImuIntrinsics *in, std::uint8_t *data, bool
    set_size);

std::size_t to_data(const Extrinsics *ex, std::uint8_t *data);

}  // namespace bytes

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_CHANNEL_BYTES_H_
