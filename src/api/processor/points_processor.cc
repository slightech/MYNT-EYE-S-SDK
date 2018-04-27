#include "api/processor/points_processor.h"

#include <glog/logging.h>

MYNTEYE_BEGIN_NAMESPACE

PointsProcessor::PointsProcessor() : Processor() {
  VLOG(2) << __func__;
}

PointsProcessor::~PointsProcessor() {
  VLOG(2) << __func__;
}

std::string PointsProcessor::Name() {
  return NAME;
}

Object *PointsProcessor::OnCreateOutput() {
  return nullptr;
}

void PointsProcessor::OnProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(in)
  UNUSED(out)
  UNUSED(parent)
}

MYNTEYE_END_NAMESPACE
