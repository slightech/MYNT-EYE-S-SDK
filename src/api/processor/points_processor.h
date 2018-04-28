#ifndef MYNTEYE_POINTS_PROCESSOR_H_  // NOLINT
#define MYNTEYE_POINTS_PROCESSOR_H_
#pragma once

#include <opencv2/core/core.hpp>

#include <string>

#include "api/processor/processor.h"

MYNTEYE_BEGIN_NAMESPACE

class PointsProcessor : public Processor {
 public:
  static constexpr auto &&NAME = "PointsProcessor";

  explicit PointsProcessor(cv::Mat Q);
  virtual ~PointsProcessor();

  std::string Name() override;

 protected:
  Object *OnCreateOutput() override;
  bool OnProcess(
      Object *const in, Object *const out, Processor *const parent) override;

 private:
  cv::Mat Q_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_POINTS_PROCESSOR_H_  NOLINT
