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

#include "mynteye/api/config.h"

MYNTEYE_BEGIN_NAMESPACE
/**
 * default intrinsics
 */

std::shared_ptr<IntrinsicsBase> getDefaultIntrinsics() {
  auto res = std::make_shared<IntrinsicsPinhole>();
  res->width = 640;
  res->height = 400;
  res->model = 0;
  res->fx = 3.6220059643202876e+02;
  res->fy = 3.6350065250745848e+02;
  res->cx = 4.0658699068023441e+02;
  res->cy = 2.3435161110061483e+02;
  double codffs[5] = {
    -2.5034765682756088e-01,
    5.0579399202897619e-02,
    -7.0536676161976066e-04,
    -8.5255451307033846e-03,
    0.
  };
  for (unsigned int i = 0; i < 5; i++) {
    res->coeffs[i] = codffs[i];
  }
  return res;
}

std::shared_ptr<Extrinsics> getDefaultExtrinsics() {
  auto res = std::make_shared<Extrinsics>();
  double rotation[9] = {
    9.9867908939669447e-01,  -6.3445566137485428e-03, 5.0988459509619687e-02,
    5.9890316389333252e-03,  9.9995670037792639e-01,  7.1224201868366971e-03,
    -5.1031440326695092e-02, -6.8076406092671274e-03, 9.9867384471984544e-01
  };
  double translation[3] = {-1.2002489764113250e+02, -1.1782637409050747e+00,
      -5.2058205159996538e+00};
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      res->rotation[i][j] = rotation[i*3 + j];
    }
  }
  for (unsigned int i = 0; i < 3; i++) {
    res->translation[i] = translation[i];
  }
  return res;
}

// sgbm_matcher = cv::StereoSGBM::create(0, 16, 3);
// sgbm_matcher->setPreFilterCap(63);
// sgbm_matcher->setBlockSize(sgbmWinSize);
// sgbm_matcher->setP1(8 * sgbmWinSize * sgbmWinSize);
// sgbm_matcher->setP2(32 * sgbmWinSize * sgbmWinSize);
// sgbm_matcher->setMinDisparity(0);
// sgbm_matcher->setNumDisparities(numberOfDisparities);
// sgbm_matcher->setUniquenessRatio(10);
// sgbm_matcher->setSpeckleWindowSize(100);
// sgbm_matcher->setSpeckleRange(32);
// sgbm_matcher->setDisp12MaxDiff(1);

// bm_matcher = cv::StereoBM::create(0, 3);
// bm_matcher->setPreFilterSize(9);
// bm_matcher->setPreFilterCap(31);
// bm_matcher->setBlockSize(15);
// bm_matcher->setMinDisparity(0);
// bm_matcher->setNumDisparities(64);
// bm_matcher->setUniquenessRatio(60);
// bm_matcher->setTextureThreshold(10);
// bm_matcher->setSpeckleWindowSize(100);
// bm_matcher->setSpeckleRange(4);


MYNTEYE_END_NAMESPACE
