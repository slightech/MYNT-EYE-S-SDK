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
#ifndef CAMERAFACTORY_H
#define CAMERAFACTORY_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

#include "camodocal/camera_models/Camera.h"

namespace camodocal {
class CameraFactory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CameraFactory();

  static boost::shared_ptr<CameraFactory> instance(void);

  CameraPtr generateCamera(
      Camera::ModelType modelType, const std::string &cameraName,
      cv::Size imageSize) const;

  CameraPtr generateCameraFromYamlFile(const std::string &filename);

 private:
  static boost::shared_ptr<CameraFactory> m_instance;
};
}  // namespace camodocal

#endif  // CAMERAFACTORY_H
