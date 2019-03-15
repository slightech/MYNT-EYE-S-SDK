#pragma once

#include "mynteye/device/callbacks.h"

#include "frame.hpp"
#include "type_conversion.hpp"

namespace mynteye_jni {

class FrameImpl : public Frame {
 public:
  using frame_t = std::shared_ptr<MYNTEYE_NAMESPACE::device::Frame>;

  explicit FrameImpl(const frame_t& frame) : frame_(frame) {}
  ~FrameImpl() {}

  /** Get the width */
  int32_t Width() override {
    return frame_->width();
  }

  /** Get the height */
  int32_t Height() override {
    return frame_->height();
  }

  /** Get the pixel format */
  ::mynteye_jni::Format Format() override {
    return to_jni(frame_->format());
  }

  /** Get the size */
  int32_t Size() override {
    return frame_->size();
  }

  /** Get the data */
  std::vector<uint8_t> Data() override {
    return std::vector<uint8_t>(frame_->data(), frame_->data() + frame_->size());
  }

  frame_t RawFrame() const {
    return frame_;
  }

 private:
  frame_t frame_;
};

}  // namespace mynteye_jni
