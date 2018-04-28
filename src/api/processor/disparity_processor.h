#ifndef MYNTEYE_DISPARITY_PROCESSOR_H_  // NOLINT
#define MYNTEYE_DISPARITY_PROCESSOR_H_
#pragma once

#include <string>

#include "api/processor/processor.h"

namespace cv {

class StereoSGBM;

}  // namespace cv

MYNTEYE_BEGIN_NAMESPACE

class DisparityProcessor : public Processor {
 public:
  static constexpr auto &&NAME = "DisparityProcessor";

  DisparityProcessor();
  virtual ~DisparityProcessor();

  std::string Name() override;

 protected:
  Object *OnCreateOutput() override;
  void OnProcess(
      Object *const in, Object *const out, Processor *const parent) override;

 private:
  cv::Ptr<cv::StereoSGBM> sgbm_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DISPARITY_PROCESSOR_H_  NOLINT
