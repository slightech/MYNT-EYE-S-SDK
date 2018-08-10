// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "api/processor/points_processor.h"

#include <opencv2/calib3d/calib3d.hpp>

#include <glog/logging.h>

#include <utility>

MYNTEYE_BEGIN_NAMESPACE

const char PointsProcessor::NAME[] = "PointsProcessor";

PointsProcessor::PointsProcessor(cv::Mat Q, std::int32_t proc_period)
    : Processor(std::move(proc_period)), Q_(std::move(Q)) {
  VLOG(2) << __func__ << ": proc_period=" << proc_period;
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
  // cv::reprojectImageTo3D(input->value, output->value, Q_, true, -1);
	bool handleMissingValues = true;
	cv::Mat disparity = input->value;
	output->value.create(disparity.size(), CV_MAKETYPE(CV_32FC3, 3));
	cv::Mat _3dImage = output->value;

	const float bigZ = 10000.f;
	cv::Matx44d Q;
	Q_.convertTo(Q, CV_64F);

	int x, cols = disparity.cols;
	CV_Assert(cols >= 0);

	double minDisparity = FLT_MAX;

	if (handleMissingValues) {
		cv::minMaxIdx(disparity, &minDisparity, 0, 0, 0);
	}

	for (int y = 0; y <	disparity.rows; y++) {

		float *sptr = disparity.ptr<float>(y);
		cv::Vec3f *dptr = _3dImage.ptr<cv::Vec3f>(y);

		for( x = 0; x < cols; x++)
		{
			double d = sptr[x];
			cv::Vec4d homg_pt = Q*cv::Vec4d(x, y, d, 1.0);
			dptr[x] = cv::Vec3d(homg_pt.val);
			dptr[x] /= homg_pt[3];

			if( fabs(d-minDisparity) <= FLT_EPSILON ) {
				dptr[x][2] = bigZ;
			}
		}
	}
  return true;
}

MYNTEYE_END_NAMESPACE
