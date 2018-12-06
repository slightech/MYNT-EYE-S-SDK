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
#include "camodocal/camera_models/CameraFactory.h"

#include <boost/algorithm/string.hpp>

#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "camodocal/camera_models/ScaramuzzaCamera.h"

#include "ceres/ceres.h"

namespace camodocal {

boost::shared_ptr<CameraFactory> CameraFactory::m_instance;

CameraFactory::CameraFactory() {}

boost::shared_ptr<CameraFactory> CameraFactory::instance(void) {
  if (m_instance.get() == 0) {
    m_instance.reset(new CameraFactory);
  }

  return m_instance;
}

CameraPtr CameraFactory::generateCamera(
    Camera::ModelType modelType, const std::string &cameraName,
    cv::Size imageSize) const {
  switch (modelType) {
    case Camera::KANNALA_BRANDT: {
      EquidistantCameraPtr camera(new EquidistantCamera);

      EquidistantCamera::Parameters params = camera->getParameters();
      params.cameraName() = cameraName;
      params.imageWidth() = imageSize.width;
      params.imageHeight() = imageSize.height;
      camera->setParameters(params);
      return camera;
    }
    case Camera::PINHOLE: {
      PinholeCameraPtr camera(new PinholeCamera);

      PinholeCamera::Parameters params = camera->getParameters();
      params.cameraName() = cameraName;
      params.imageWidth() = imageSize.width;
      params.imageHeight() = imageSize.height;
      camera->setParameters(params);
      return camera;
    }
    case Camera::SCARAMUZZA: {
      OCAMCameraPtr camera(new OCAMCamera);

      OCAMCamera::Parameters params = camera->getParameters();
      params.cameraName() = cameraName;
      params.imageWidth() = imageSize.width;
      params.imageHeight() = imageSize.height;
      camera->setParameters(params);
      return camera;
    }
    case Camera::MEI:
    default: {
      CataCameraPtr camera(new CataCamera);

      CataCamera::Parameters params = camera->getParameters();
      params.cameraName() = cameraName;
      params.imageWidth() = imageSize.width;
      params.imageHeight() = imageSize.height;
      camera->setParameters(params);
      return camera;
    }
  }
}

CameraPtr CameraFactory::generateCameraFromYamlFile(
    const std::string &filename) {
  cv::FileStorage fs(filename, cv::FileStorage::READ);

  if (!fs.isOpened()) {
    std::cout << "# ERROR: can not open " << filename << std::endl;
    return CameraPtr();
  }

  Camera::ModelType modelType = Camera::MEI;
  if (!fs["model_type"].isNone()) {
    std::string sModelType;
    fs["model_type"] >> sModelType;

    if (boost::iequals(sModelType, "kannala_brandt")) {
      modelType = Camera::KANNALA_BRANDT;
    } else if (boost::iequals(sModelType, "mei")) {
      modelType = Camera::MEI;
    } else if (boost::iequals(sModelType, "scaramuzza")) {
      modelType = Camera::SCARAMUZZA;
    } else if (boost::iequals(sModelType, "pinhole")) {
      modelType = Camera::PINHOLE;
    } else {
      std::cerr << "# ERROR: Unknown camera model: " << sModelType << std::endl;
      return CameraPtr();
    }
  }

  switch (modelType) {
    case Camera::KANNALA_BRANDT: {
      EquidistantCameraPtr camera(new EquidistantCamera);

      EquidistantCamera::Parameters params = camera->getParameters();
      params.readFromYamlFile(filename);
      camera->setParameters(params);
      return camera;
    }
    case Camera::PINHOLE: {
      PinholeCameraPtr camera(new PinholeCamera);

      PinholeCamera::Parameters params = camera->getParameters();
      params.readFromYamlFile(filename);
      camera->setParameters(params);
      return camera;
    }
    case Camera::SCARAMUZZA: {
      OCAMCameraPtr camera(new OCAMCamera);

      OCAMCamera::Parameters params = camera->getParameters();
      params.readFromYamlFile(filename);
      camera->setParameters(params);
      return camera;
    }
    case Camera::MEI:
    default: {
      CataCameraPtr camera(new CataCamera);

      CataCamera::Parameters params = camera->getParameters();
      params.readFromYamlFile(filename);
      camera->setParameters(params);
      return camera;
    }
  }

  return CameraPtr();
}
}  // namespace camodocal
