#include "api/processor/disparity_normalized_processor.h"

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
  return nullptr;
}

void DisparityNormalizedProcessor::OnProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(in)
  UNUSED(out)
  UNUSED(parent)
}

MYNTEYE_END_NAMESPACE
