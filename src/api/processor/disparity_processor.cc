#include "api/processor/disparity_processor.h"

#include <opencv2/calib3d/calib3d.hpp>

#include <glog/logging.h>

MYNTEYE_BEGIN_NAMESPACE

DisparityProcessor::DisparityProcessor() : Processor() {
  VLOG(2) << __func__;
  int sgbmWinSize = 3;
  int numberOfDisparities = 64;

#ifdef USE_OPENCV2
  // StereoSGBM
  //   http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html?#stereosgbm
  sgbm_ = cv::Ptr<cv::StereoSGBM>(
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
  sgbm_ = cv::StereoSGBM::create(0, 16, 3);
  sgbm_->setPreFilterCap(63);
  sgbm_->setBlockSize(sgbmWinSize);
  sgbm_->setP1(8 * sgbmWinSize * sgbmWinSize);
  sgbm_->setP2(32 * sgbmWinSize * sgbmWinSize);
  sgbm_->setMinDisparity(0);
  sgbm_->setNumDisparities(numberOfDisparities);
  sgbm_->setUniquenessRatio(10);
  sgbm_->setSpeckleWindowSize(100);
  sgbm_->setSpeckleRange(32);
  sgbm_->setDisp12MaxDiff(1);
#endif
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
  UNUSED(parent)
  const ObjMat2 *input = Object::Cast<ObjMat2>(in);
  ObjMat *output = Object::Cast<ObjMat>(out);

  cv::Mat disparity;
#ifdef USE_OPENCV2
  // StereoSGBM::operator()
  //   http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereosgbm-operator
  // Output disparity map. It is a 16-bit signed single-channel image of the
  // same size as the input image.
  // It contains disparity values scaled by 16. So, to get the floating-point
  // disparity map,
  // you need to divide each disp element by 16.
  (*sgbm_)(input->first, input->second, disparity);
#else
  // compute()
  //   http://docs.opencv.org/master/d2/d6e/classcv_1_1StereoMatcher.html
  // Output disparity map. It has the same size as the input images.
  // Some algorithms, like StereoBM or StereoSGBM compute 16-bit fixed-point
  // disparity map
  // (where each disparity value has 4 fractional bits),
  // whereas other algorithms output 32-bit floating-point disparity map.
  sgbm_->compute(input->first, input->second, disparity);
#endif
  output->value = disparity / 16 + 1;
  return true;
}

MYNTEYE_END_NAMESPACE
