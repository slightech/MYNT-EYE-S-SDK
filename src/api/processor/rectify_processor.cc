#include "api/processor/rectify_processor.h"

#include <glog/logging.h>

MYNTEYE_BEGIN_NAMESPACE

RectifyProcessor::RectifyProcessor() : Processor() {
  VLOG(2) << __func__;
}

RectifyProcessor::~RectifyProcessor() {
  VLOG(2) << __func__;
}

std::string RectifyProcessor::Name() {
  return NAME;
}

Object *RectifyProcessor::OnCreateOutput() {
  return nullptr;
}

void RectifyProcessor::OnProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(in)
  UNUSED(out)
  UNUSED(parent)
}

MYNTEYE_END_NAMESPACE
