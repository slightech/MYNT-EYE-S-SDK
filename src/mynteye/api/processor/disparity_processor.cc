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
#ifndef WITH_OPENCV2
#include <opencv2/core/persistence.hpp>
#endif
#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

const char DisparityProcessor::NAME[] = "DisparityProcessor";

DisparityProcessor::DisparityProcessor(DisparityComputingMethod type,
    std::shared_ptr<struct CameraROSMsgInfoPair> calib_infos,
    std::int32_t proc_period)
    : Processor(std::move(proc_period)), type_(type) {
  if (calib_infos) {
    cx1_minus_cx2_ = calib_infos->cx1_minus_cx2;
  } else {
    cx1_minus_cx2_ = 1.f;
  }
  VLOG(2) << __func__ << ": proc_period=" << proc_period;
  int sgbmWinSize = 3;
  int numberOfDisparities = 128;
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
    LOG(ERROR) << "BM not supported in opencv 2.x, use sgbm";
    // int bmWinSize = 3;
    // // StereoBM
    // //   https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereobm-stereobm
    // bm_matcher = cv::Ptr<cv::StereoBM>(new cv::StereoBM(
    //     int 0,
    //     64,
    //     100,
    //     8 * bmWinSize * bmWinSize,
    //     32 * bmWinSize * bmWinSize,
    //     int -1,
    //     int 31,
    //     15,
    //     100,
    //     4));
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
    disparity_min_sgbm_ptr =
      std::make_shared<int>(sgbm_matcher->getMinDisparity());
    disparity_max_sgbm_ptr =
      std::make_shared<int>(sgbm_matcher->getNumDisparities());
    bm_matcher = cv::StereoBM::create(0, 3);
    bm_matcher->setPreFilterSize(9);
    bm_matcher->setPreFilterCap(31);
    bm_matcher->setBlockSize(15);
    bm_matcher->setMinDisparity(0);
    bm_matcher->setNumDisparities(numberOfDisparities);
    bm_matcher->setUniquenessRatio(60);
    bm_matcher->setTextureThreshold(10);
    bm_matcher->setSpeckleWindowSize(100);
    bm_matcher->setSpeckleRange(4);
    bm_matcher->setPreFilterType(cv::StereoBM::PREFILTER_XSOBEL);
    disparity_min_bm_ptr =
      std::make_shared<int>(bm_matcher->getMinDisparity());
    disparity_max_bm_ptr =
      std::make_shared<int>(bm_matcher->getNumDisparities());
#endif
  NotifyComputingTypeChanged(type_);
}

void DisparityProcessor::NotifyComputingTypeChanged(
    const DisparityComputingMethod &MethodType) {
  type_ = MethodType;
}

DisparityProcessor::~DisparityProcessor() {
  VLOG(2) << __func__;
}

void DisparityProcessor::SetDisparityComputingMethodType(
    const DisparityComputingMethod &MethodType) {
  NotifyComputingTypeChanged(MethodType);
}

bool DisparityProcessor::ConfigFromFile(const std::string& config_file) {
#ifndef WITH_OPENCV2
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
      std::cerr << "ERROR: Wrong path to settings" << std::endl;
      return false;
  }
  cv::FileNode node_sgbm = fsSettings["SGBM"];
  if (node_sgbm.type() == cv::FileNode::MAP) {
    sgbm_matcher->read(node_sgbm);
    *disparity_min_sgbm_ptr = sgbm_matcher->getMinDisparity();
    *disparity_max_sgbm_ptr = sgbm_matcher->getNumDisparities();
  }

  cv::FileNode node_bm = fsSettings["BM"];
  if (node_bm.type() == cv::FileNode::MAP) {
    bm_matcher->read(node_bm);
    *disparity_min_bm_ptr = bm_matcher->getMinDisparity();
    *disparity_max_bm_ptr = bm_matcher->getNumDisparities();
  }
  return true;
#else
  return false;
#endif
}

std::string DisparityProcessor::Name() {
  return NAME;
}

Object *DisparityProcessor::OnCreateOutput() {
  return new ObjMat();
}

bool DisparityProcessor::OnProcess(
    Object *const in, Object *const out,
    std::shared_ptr<Processor> const parent) {
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
  if (type_ == DisparityComputingMethod::SGBM) {
    (*sgbm_matcher)(input->first, input->second, disparity);
  } else if (type_ == DisparityComputingMethod::BM) {
    // LOG(ERROR) << "not supported in opencv 2.x";
    (*sgbm_matcher)(input->first, input->second, disparity);
    disparity.convertTo(output->value, CV_32F, 1./16, 1);
    // cv::Mat tmp1, tmp2;
    // cv::cvtColor(input->first, tmp1, CV_RGB2GRAY);
    // cv::cvtColor(input->second, tmp2, CV_RGB2GRAY);
    // (*bm_matcher)(tmp1, tmp2, disparity);
  }
#else
  // compute()
  //   http://docs.opencv.org/master/d2/d6e/classcv_1_1StereoMatcher.html
  // Output disparity map. It has the same size as the input images.
  // Some algorithms, like StereoBM or StereoSGBM compute 16-bit fixed-point
  // disparity map
  // (where each disparity value has 4 fractional bits),
  // whereas other algorithms output 32-bit floating-point disparity map.
  if (type_ == DisparityComputingMethod::SGBM) {
    sgbm_matcher->compute(input->first, input->second, disparity);
  } else if (type_ == DisparityComputingMethod::BM) {
    cv::Mat tmp1, tmp2;
    if (input->first.channels() == 1) {
      // s1030
      tmp1 = input->first;
      tmp2 = input->second;
    } else if (input->first.channels() >= 3) {
      // s210
      cv::cvtColor(input->first, tmp1, cv::COLOR_RGB2GRAY);
      cv::cvtColor(input->second, tmp2, cv::COLOR_RGB2GRAY);
    }
    bm_matcher->compute(tmp1, tmp2, disparity);
  } else {
    // default
    sgbm_matcher->compute(input->first, input->second, disparity);
  }
#endif
  disparity.convertTo(output->value, CV_32F, 1./16, cx1_minus_cx2_);
  output->id = input->first_id;
  output->data = input->first_data;
  return true;
}

MYNTEYE_END_NAMESPACE
