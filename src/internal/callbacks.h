#ifndef MYNTEYE_INTERNAL_CALLBACKS_H_  // NOLINT
#define MYNTEYE_INTERNAL_CALLBACKS_H_
#pragma once

#include <cstdint>
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
    const std::uint8_t *bytes = static_cast<const std::uint8_t *>(data);
    data_ = data_t(bytes, bytes + (width * height) * bytes_per_pixel(format));
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

  const data_t &data() const {
    return data_;
  }

 private:
  std::uint16_t width_;
  std::uint16_t height_;
  Format format_;

  data_t data_;
};

struct MYNTEYE_API StreamData {
  ImgData img;
  std::shared_ptr<Frame> frame;
};

struct MYNTEYE_API MotionData {
  ImuData imu;
};

using StreamCallback = std::function<void(const StreamData &data)>;
using MotionCallback = std::function<void(const MotionData &data)>;

}  // namespace device

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_CALLBACKS_H_ NOLINT
