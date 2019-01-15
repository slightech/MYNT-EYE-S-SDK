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
#include "mynteye/api/processor/disparity_processor.h"

#include <utility>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#ifdef WITH_BM_SOBEL_FILTER
#include <opencv2/ximgproc/disparity_filter.hpp>
#endif

#include "mynteye/logger.h"

#define WITH_BM_SOBEL_FILTER

MYNTEYE_BEGIN_NAMESPACE

const char DisparityProcessor::NAME[] = "DisparityProcessor";

DisparityProcessor::DisparityProcessor(DisparityProcessorType type,
    std::int32_t proc_period)
    : Processor(std::move(proc_period)), type_(type) {
  VLOG(2) << __func__ << ": proc_period=" << proc_period;

  if (type_ == DisparityProcessorType::SGBM) {
    int sgbmWinSize = 3;
    int numberOfDisparities = 64;

#ifdef WITH_OPENCV2
    // StereoSGBM
    //   http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html?#stereosgbm
    sgbm_matcher = cv::Ptr<cv::StereoSGBM>(
        new cv::StereoSGBM(
            0,                               // minDisparity
            numberOfDisparities,             // numDisparities
            sgbmWinSize,                     // SADWindowSize
            8 * sgbmWinSize * sgbmWinSize,   // P1
            32 * sgbmWinSize * sgbmWinSize,  // P2
            1,                               // disp12MaxDiff
            63,                              // preFilterCap
            10,                              // uniquenessRatio
            100,                             // speckleWindowSize
            32,                              // speckleRange
            false));                         // fullDP
#else
    sgbm_matcher = cv::StereoSGBM::create(0, 16, 3);
    sgbm_matcher->setPreFilterCap(63);
    sgbm_matcher->setBlockSize(sgbmWinSize);
    sgbm_matcher->setP1(8 * sgbmWinSize * sgbmWinSize);
    sgbm_matcher->setP2(32 * sgbmWinSize * sgbmWinSize);
    sgbm_matcher->setMinDisparity(0);
    sgbm_matcher->setNumDisparities(numberOfDisparities);
    sgbm_matcher->setUniquenessRatio(10);
    sgbm_matcher->setSpeckleWindowSize(100);
    sgbm_matcher->setSpeckleRange(32);
    sgbm_matcher->setDisp12MaxDiff(1);
#endif
#ifdef WITH_BM_SOBEL_FILTER
  } else if (type_ == DisparityProcessorType::BM) {
    int bmWinSize = 3;
#ifdef WITH_OPENCV2
    int bmWinSize = 3;
    // StereoBM
    //   https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereobm-stereobm
    bm_matcher = cv::Ptr<cv::StereoBM>(new cv::StereoBM(
        int 0,
        64,
        100,
        8 * bmWinSize * bmWinSize,
        32 * bmWinSize * bmWinSize,
        int -1,
        int 31,
        15,
        100,
        4));
#else
    bm_matcher = cv::StereoBM::create(0, 3);
    bm_matcher->setPreFilterSize(9);
    bm_matcher->setPreFilterCap(31);
    bm_matcher->setBlockSize(bmWinSize);
    bm_matcher->setMinDisparity(0);
    bm_matcher->setNumDisparities(64);
    bm_matcher->setUniquenessRatio(15);
    bm_matcher->setTextureThreshold(10);
    bm_matcher->setSpeckleWindowSize(100);
    bm_matcher->setSpeckleRange(4);
    bm_matcher->setPreFilterType(cv::StereoBM::PREFILTER_XSOBEL);
#endif
#endif
  } else {
    LOG(ERROR) << "no enum DisparityProcessorType,use default sgbm";
    int sgbmWinSize = 3;
    int numberOfDisparities = 64;

#ifdef WITH_OPENCV2
    // StereoSGBM
    //   http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html?#stereosgbm
    sgbm_matcher = cv::Ptr<cv::StereoSGBM>(
        new cv::StereoSGBM(
            0,                               // minDisparity
            numberOfDisparities,             // numDisparities
            sgbmWinSize,                     // SADWindowSize
            8 * sgbmWinSize * sgbmWinSize,   // P1
            32 * sgbmWinSize * sgbmWinSize,  // P2
            1,                               // disp12MaxDiff
            63,                              // preFilterCap
            10,                              // uniquenessRatio
            100,                             // speckleWindowSize
            32,                              // speckleRange
            false));                         // fullDP
#else
    sgbm_matcher = cv::StereoSGBM::create(0, 16, 3);
    sgbm_matcher->setPreFilterCap(63);
    sgbm_matcher->setBlockSize(sgbmWinSize);
    sgbm_matcher->setP1(8 * sgbmWinSize * sgbmWinSize);
    sgbm_matcher->setP2(32 * sgbmWinSize * sgbmWinSize);
    sgbm_matcher->setMinDisparity(0);
    sgbm_matcher->setNumDisparities(numberOfDisparities);
    sgbm_matcher->setUniquenessRatio(10);
    sgbm_matcher->setSpeckleWindowSize(100);
    sgbm_matcher->setSpeckleRange(32);
    sgbm_matcher->setDisp12MaxDiff(1);
#endif
  }
}

DisparityProcessor::~DisparityProcessor() {
  VLOG(2) << __func__;
}

std::string DisparityProcessor::Name() {
  return NAME;
}

Object *DisparityProcessor::OnCreateOutput() {
  return new ObjMat();
}

bool DisparityProcessor::OnProcess(
    Object *const in, Object *const out, Processor *const parent) {
  MYNTEYE_UNUSED(parent)
  const ObjMat2 *input = Object::Cast<ObjMat2>(in);
  ObjMat *output = Object::Cast<ObjMat>(out);

  cv::Mat disparity;
#ifdef WITH_OPENCV2
  // StereoSGBM::operator()
  //   http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereosgbm-operator
  // Output disparity map. It is a 16-bit signed single-channel image of the
  // same size as the input image.
  // It contains disparity values scaled by 16. So, to get the floating-point
  // disparity map,
  // you need to divide each disp element by 16.
  if (type_ == DisparityProcessorType::SGBM) {
    (*sgbm_matcher)(input->first, input->second, disparity);
#ifdef WITH_BM_SOBEL_FILTER
  } else if (type_ == DisparityProcessorType::BM) {
    cv::Mat tmp1, tmp2;
    cv::cvtColor(input->first, tmp1, CV_RGB2GRAY);
    cv::cvtColor(input->second, tmp2, CV_RGB2GRAY);
    (*bm_matcher)(tmp1, tmp2, disparity);
#endif
  }
#else
  // compute()
  //   http://docs.opencv.org/master/d2/d6e/classcv_1_1StereoMatcher.html
  // Output disparity map. It has the same size as the input images.
  // Some algorithms, like StereoBM or StereoSGBM compute 16-bit fixed-point
  // disparity map
  // (where each disparity value has 4 fractional bits),
  // whereas other algorithms output 32-bit floating-point disparity map.
  if (type_ == DisparityProcessorType::SGBM) {
    sgbm_matcher->compute(input->first, input->second, disparity);
#ifdef WITH_BM_SOBEL_FILTER
  } else if (type_ == DisparityProcessorType::BM) {
    cv::Mat tmp1, tmp2;
    if (input->first.channels() == 1) {
      // s1030
    } else if (input->first.channels() == 3) {
      // s210
      cv::cvtColor(input->first, tmp1, CV_RGB2GRAY);
      cv::cvtColor(input->second, tmp2, CV_RGB2GRAY);
    }
    bm_matcher->compute(tmp1, tmp2, disparity);
#endif
  } else {
    // default
    sgbm_matcher->compute(input->first, input->second, disparity);
  }
#endif
  disparity.convertTo(output->value, CV_32F, 1./16, 1);
  output->id = input->first_id;
  output->data = input->first_data;
  return true;
}

MYNTEYE_END_NAMESPACE
