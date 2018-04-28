#include "api/processor/disparity_normalized_processor.h"

#include <opencv2/imgproc/imgproc.hpp>

#include <glog/logging.h>

MYNTEYE_BEGIN_NAMESPACE

DisparityNormalizedProcessor::DisparityNormalizedProcessor() : Processor() {
  VLOG(2) << __func__;
}

DisparityNormalizedProcessor::~DisparityNormalizedProcessor() {
  VLOG(2) << __func__;
}

std::string DisparityNormalizedProcessor::Name() {
  return NAME;
}

Object *DisparityNormalizedProcessor::OnCreateOutput() {
  return new ObjMat();
}

bool DisparityNormalizedProcessor::OnProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(parent)
  const ObjMat *input = Object::Cast<ObjMat>(in);
  ObjMat *output = Object::Cast<ObjMat>(out);
  cv::normalize(input->value, output->value, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  // cv::normalize maybe return empty ==
  return !output->value.empty();
}

MYNTEYE_END_NAMESPACE
