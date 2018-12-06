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
#include <iostream>
#include <vector>
#include<opencv2/core/core.hpp>
namespace mynteye_image_pipeline
{
namespace tools {
  // read camera extrinsics data from camodocal extrinsic.yaml
  // [q_x, q_y, q_z, q_w, t_x, t_y, t_z]
  const std::vector<double> readExtrinsicsFrom(const std::string &filename);
}

namespace data_conveter {
  // q_x, q_y, q_z, q_w
  void solveR1R2(double extrintics[7], cv::Mat& R1_cv, cv::Mat& R2_cv);
}
}  // namespace mynteye_image_pipeline
