#include "api/processor/depth_processor.h"

#include <glog/logging.h>

MYNTEYE_BEGIN_NAMESPACE

DepthProcessor::DepthProcessor() : Processor() {
  VLOG(2) << __func__;
}

DepthProcessor::~DepthProcessor() {
  VLOG(2) << __func__;
}

std::string DepthProcessor::Name() {
  return NAME;
}

Object *DepthProcessor::OnCreateOutput() {
  return nullptr;
}

bool DepthProcessor::OnProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(in)
  UNUSED(out)
  UNUSED(parent)
  return true;
}

MYNTEYE_END_NAMESPACE
