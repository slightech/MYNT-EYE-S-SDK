#pragma once

#include "mynteye/device/callbacks.h"

#include "frame_impl.hpp"
#include "img_data.hpp"
#include "stream_data.hpp"

namespace mynteye_jni {

class StreamDataImpl : public StreamData {
 public:
  using stream_data_t = MYNTEYE_NAMESPACE::device::StreamData;

  explicit StreamDataImpl(const stream_data_t& data) : data_(data) {}
  ~StreamDataImpl() {}

  ImgData Img() override {
    auto&& img = data_.img;
    return {
      img->frame_id,
      static_cast<int64_t>(img->timestamp),
      img->exposure_time,
    };
  }

  std::shared_ptr<::mynteye_jni::Frame> Frame() override {
    return std::make_shared<::mynteye_jni::FrameImpl>(data_.frame);
  }

  int64_t FrameId() override {
    return data_.frame_id;
  }

 private:
  stream_data_t data_;
};

}  // namespace mynteye_jni
