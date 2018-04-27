#ifndef MYNTEYE_RECTIFY_PROCESSOR_H_  // NOLINT
#define MYNTEYE_RECTIFY_PROCESSOR_H_
#pragma once

#include <string>

#include "api/processor/processor.h"

MYNTEYE_BEGIN_NAMESPACE

class RectifyProcessor : public Processor {
 public:
  static constexpr auto &&NAME = "RectifyProcessor";

  RectifyProcessor();
  virtual ~RectifyProcessor();

  std::string Name() override;

 protected:
  Object *OnCreateOutput() override;
  void OnProcess(
      Object *const in, Object *const out, Processor *const parent) override;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_RECTIFY_PROCESSOR_H_  NOLINT
