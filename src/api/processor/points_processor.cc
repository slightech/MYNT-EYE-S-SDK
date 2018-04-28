#include "api/processor/points_processor.h"

#include <opencv2/calib3d/calib3d.hpp>

#include <glog/logging.h>

#include <utility>

MYNTEYE_BEGIN_NAMESPACE

PointsProcessor::PointsProcessor(cv::Mat Q) : Processor(), Q_(std::move(Q)) {
  VLOG(2) << __func__;
}

PointsProcessor::~PointsProcessor() {
  VLOG(2) << __func__;
}

std::string PointsProcessor::Name() {
  return NAME;
}

Object *PointsProcessor::OnCreateOutput() {
  return new ObjMat();
}

bool PointsProcessor::OnProcess(
    Object *const in, Object *const out, Processor *const parent) {
  UNUSED(parent)
  const ObjMat *input = Object::Cast<ObjMat>(in);
  ObjMat *output = Object::Cast<ObjMat>(out);
  cv::reprojectImageTo3D(input->value, output->value, Q_, true);
  return true;
}

MYNTEYE_END_NAMESPACE
