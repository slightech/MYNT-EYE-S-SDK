#include "api/processor/disparity_processor.h"

#include <glog/logging.h>

MYNTEYE_BEGIN_NAMESPACE

DisparityProcessor::DisparityProcessor() : Processor() {
  VLOG(2) << __func__;
}

DisparityProcessor::~DisparityProcessor() {
  VLOG(2) << __func__;
}

std::string DisparityProcessor::Name() {
  return NAME;
}

Object *DisparityProcessor::OnCreateOutput() {
  return nullptr;
}

void DisparityProcessor::OnProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(in)
  UNUSED(out)
  UNUSED(parent)
}

MYNTEYE_END_NAMESPACE
