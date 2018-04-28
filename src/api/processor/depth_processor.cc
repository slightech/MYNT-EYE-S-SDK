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
  return new ObjMat();
}

bool DepthProcessor::OnProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(parent)
  const ObjMat *input = Object::Cast<ObjMat>(in);
  ObjMat *output = Object::Cast<ObjMat>(out);
  cv::Mat channels[3 /*input->value.channels()*/];
  cv::split(input->value, channels);
  channels[2].convertTo(output->value, CV_16UC1);
  return true;
}

MYNTEYE_END_NAMESPACE
