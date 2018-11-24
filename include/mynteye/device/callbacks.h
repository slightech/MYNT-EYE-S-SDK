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
#ifndef MYNTEYE_DEVICE_CALLBACKS_H_
#define MYNTEYE_DEVICE_CALLBACKS_H_
#pragma once

#include <cstdint>

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

#include "mynteye/mynteye.h"
#include "mynteye/types.h"

MYNTEYE_BEGIN_NAMESPACE

namespace device {

/**
 * @ingroup datatypes
 * Frame with raw data.
 */
class MYNTEYE_API Frame {
 public:
  using data_t = std::vector<std::uint8_t>;

  /**
   * Construct the frame with StreamRequest and raw data.
   */
  Frame(const StreamRequest &request, const void *data)
      : Frame(request.width, request.height, request.format, data) {}

  /**
   * Construct the frame with stream info and raw data.
   */
  Frame(
      std::uint16_t width, std::uint16_t height, Format format,
      const void *data)
      : width_(width), height_(height), format_(format) {
    std::size_t bytes_n = (width * height) * bytes_per_pixel(format);
    if (data) {
      const std::uint8_t *bytes = static_cast<const std::uint8_t *>(data);
      data_ = data_t(bytes, bytes + bytes_n);
    } else {
      data_.assign(bytes_n, 0);
    }
  }

  /** Get the width. */
  std::uint16_t width() const {
    return width_;
  }

  /** Get the height. */
  std::uint16_t height() const {
    return height_;
  }

  /** Get the format. */
  Format format() const {
    return format_;
  }

  /** Get the data. */
  std::uint8_t *data() {
    return data_.data();
  }

  /** Get the const data. */
  const std::uint8_t *data() const {
    return data_.data();
  }

  /** Get the size of data. */
  std::size_t size() const {
    return data_.size();
  }

  /** Clone a new frame. */
  Frame clone() const {
    Frame frame(width_, height_, format_, nullptr);
    std::copy(data_.begin(), data_.end(), frame.data_.begin());
    return frame;
  }

 private:
  std::uint16_t width_;
  std::uint16_t height_;
  Format format_;

  data_t data_;
};

/**
 * @ingroup datatypes
 * Device stream data.
 */
struct MYNTEYE_API StreamData {
  /** ImgData. */
  std::shared_ptr<ImgData> img;
  /** Frame. */
  std::shared_ptr<Frame> frame;
  /** Frame ID. */
  std::uint16_t frame_id;
};

/**
 * @ingroup datatypes
 * Device motion data.
 */
struct MYNTEYE_API MotionData {
  /** ImuData. */
  std::shared_ptr<ImuData> imu;
};

using StreamCallback = std::function<void(const StreamData &data)>;
using MotionCallback = std::function<void(const MotionData &data)>;

}  // namespace device

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_CALLBACKS_H_
