#ifndef MYNTEYE_DISPARITY_NORMALIZED_PROCESSOR_H_  // NOLINT
#define MYNTEYE_DISPARITY_NORMALIZED_PROCESSOR_H_
#pragma once

#include <string>

#include "api/processor/processor.h"

MYNTEYE_BEGIN_NAMESPACE

class DisparityNormalizedProcessor : public Processor {
 public:
  static constexpr auto &&NAME = "DisparityNormalizedProcessor";

  DisparityNormalizedProcessor();
  virtual ~DisparityNormalizedProcessor();

  std::string Name() override;

 protected:
  Object *OnCreateOutput() override;
  bool OnProcess(
      Object *const in, Object *const out, Processor *const parent) override;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DISPARITY_NORMALIZED_PROCESSOR_H_  NOLINT
