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
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <camodocal/camera_models/CameraFactory.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>
#include <camodocal/camera_models/CataCamera.h>
#include <camodocal/camera_models/EquidistantCamera.h>
#include <camodocal/camera_models/PinholeCamera.h>
namespace mynteye_image_pipeline {
class CameraManager {
 public:
  enum CameraSide {
    LEFT,
    RIGHT
  };

 public:
  CameraManager() {}
  static boost::shared_ptr<CameraManager> Instance();

  void generateCameraFromYamlFile(const std::string& camera_yaml,
    const std::string& extrinsics_yaml);

  sensor_msgs::CameraInfoPtr generateEquCameraInfo(
    const CameraSide& side, std_msgs::Header& header);
  sensor_msgs::CameraInfoPtr generateCataCameraInfo(
    const CameraSide& side, std_msgs::Header& header);
  sensor_msgs::CameraInfoPtr generatePinholeCameraInfo(
    const CameraSide& side, std_msgs::Header& header);
  std::string toStringCameraInfoName();

  Eigen::Matrix3d solveR(const CameraSide& side);

  camodocal::CameraPtr getCamOdoCameraPtr(const sensor_msgs::CameraInfoConstPtr& camera_info_ptr);
 private:
  Eigen::Matrix4d loadT(const CameraSide& side);
  Eigen::Matrix4d generateRecifyT(const CameraSide& side);

  private:
    std::string _extrinsics_yaml;
    camodocal::CameraPtr _camera_ptr;
    static boost::shared_ptr<CameraManager> m_instance;
    std::map<CameraSide, Eigen::Matrix3d> _R;
    double _baseline;
};

class CameraManagerTools {
 public:
  static camodocal::CameraPtr getCamOdoCameraPtr(
    const sensor_msgs::CameraInfoConstPtr& camera_info_ptr);
  static sensor_msgs::CameraInfoPtr generateCameraFromYamlFile(
    const std::string &file_name);
  static sensor_msgs::CameraInfoPtr setDistortionToZereo(
    const sensor_msgs::CameraInfoConstPtr& camera_info_ptr);
 private:
  static sensor_msgs::CameraInfoPtr generatorCameraInfo(
    camodocal::EquidistantCamera::Parameters &params);
  static sensor_msgs::CameraInfoPtr generatorCameraInfo(
    camodocal::CataCamera::Parameters &params);
  static sensor_msgs::CameraInfoPtr generatorCameraInfo(
    camodocal::PinholeCamera::Parameters &params);
  };
}  // namespace mynteye_image_pipeline
