#ifndef MYNTEYE_CALLBACKS_H_  // NOLINT
#define MYNTEYE_CALLBACKS_H_
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

class Frame {
 public:
  using data_t = std::vector<std::uint8_t>;

  Frame(const StreamRequest &request, const void *data)
      : Frame(request.width, request.height, request.format, data) {}

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

  std::uint16_t width() const {
    return width_;
  }

  std::uint16_t height() const {
    return height_;
  }

  Format format() const {
    return format_;
  }

  std::uint8_t *data() {
    return data_.data();
  }

  const std::uint8_t *data() const {
    return data_.data();
  }

  std::size_t size() const {
    return data_.size();
  }

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

struct MYNTEYE_API StreamData {
  std::shared_ptr<ImgData> img;
  std::shared_ptr<Frame> frame;
};

struct MYNTEYE_API MotionData {
  std::shared_ptr<ImuData> imu;
};

using StreamCallback = std::function<void(const StreamData &data)>;
using MotionCallback = std::function<void(const MotionData &data)>;

}  // namespace device

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_CALLBACKS_H_ NOLINT
