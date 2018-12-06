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
#include <mynteye_image_pipeline/tools/tools.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
namespace mynteye_image_pipeline {
namespace tools {
const std::vector<double> readExtrinsicsFrom(const std::string &filename) {
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "#ERROR: can not open file "<< filename;
  }
  std::vector<double> extrinsics_data(7);
  extrinsics_data[0] = fs["transform"]["q_x"];
  extrinsics_data[1] = fs["transform"]["q_y"];
  extrinsics_data[2] = fs["transform"]["q_z"];
  extrinsics_data[3] = fs["transform"]["q_w"];
  extrinsics_data[4] = fs["transform"]["t_x"];
  extrinsics_data[5] = fs["transform"]["t_y"];
  extrinsics_data[6] = fs["transform"]["t_z"];
  fs.release();
  return extrinsics_data;
}
}  // namespace tools

namespace data_conveter {
  // q_x, q_y, q_z, q_w
void solveR1R2(double extrintics[7],
                cv::Mat& R1_cv,
                cv::Mat& R2_cv) {
  Eigen::Quaterniond q(
    extrintics[3], extrintics[0],
    extrintics[1], extrintics[2]);
  q.normalize();
  Eigen::Matrix4d H_cam2  = Eigen::Matrix4d::Identity();
  H_cam2.topLeftCorner<3, 3>() = q.toRotationMatrix();
  H_cam2.topRightCorner<3, 1>()
    << extrintics[4], extrintics[5], extrintics[6];
  Eigen::Matrix4d H_cam1 = Eigen::Matrix4d::Identity();
  std::cout << "H_cam2" << H_cam2 << std::endl;
  std::cout << "H_cam1: " << H_cam1 << std::endl;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
    H_cam1.block<3, 3>(0, 0) + H_cam2.block<3, 3>(0, 0),
    Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d avgR = svd.matrixU() * svd.matrixV().transpose();
  Eigen::Matrix3d R1 = avgR *
    H_cam1.block<3, 3>(0, 0).transpose();
  Eigen::Matrix3d R2 = avgR * H_cam2.block<3, 3>(0, 0).transpose();
  std::cout << "R1" << R1 << std::endl;
  std::cout << "R2" << R2 << std::endl;

  cv::eigen2cv(R1, R1_cv);
  cv::eigen2cv(R2, R2_cv);
  std::cout << "R1_cv: " << R1_cv << std::endl;
  std::cout << "R2_cv: " << R2_cv << std::endl;
}
}  // namespace data_conveter
}  // namespace mynteye_image_pipeline
