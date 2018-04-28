#ifndef MYNTEYE_RECTIFY_PROCESSOR_H_  // NOLINT
#define MYNTEYE_RECTIFY_PROCESSOR_H_
#pragma once

#include <opencv2/core/core.hpp>

#include <memory>
#include <string>

#include "api/processor/processor.h"
#include "mynteye/types.h"

MYNTEYE_BEGIN_NAMESPACE

class Device;

class RectifyProcessor : public Processor {
 public:
  static constexpr auto &&NAME = "RectifyProcessor";

  explicit RectifyProcessor(std::shared_ptr<Device> device);
  virtual ~RectifyProcessor();

  std::string Name() override;

 protected:
  Object *OnCreateOutput() override;
  void OnProcess(
      Object *const in, Object *const out, Processor *const parent) override;

 private:
  void InitParams(
      Intrinsics in_left, Intrinsics in_right, Extrinsics ex_left_to_right);

  cv::Mat R1, P1, R2, P2, Q;
  cv::Mat map11, map12, map21, map22;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_RECTIFY_PROCESSOR_H_  NOLINT
