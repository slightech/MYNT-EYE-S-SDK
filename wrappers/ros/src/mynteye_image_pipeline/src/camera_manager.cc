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
#include <mynteye_image_pipeline/camera_manager.h>
#include <mynteye_image_pipeline/tools/tools.h>
#include <boost/algorithm/string.hpp>
namespace mynteye_image_pipeline {
boost::shared_ptr<CameraManager> CameraManager::m_instance;
boost::shared_ptr<CameraManager> CameraManager::Instance() {
  if (m_instance.get() == 0) {
    m_instance.reset(new CameraManager);
  }
  return m_instance;
}
void CameraManager::generateCameraFromYamlFile(
    const std::string& camera_yaml, const std::string& extrinsics_yaml) {
  _extrinsics_yaml = extrinsics_yaml;
  _camera_ptr = camodocal::CameraFactory::instance()->
    generateCameraFromYamlFile(camera_yaml);
}

sensor_msgs::CameraInfoPtr CameraManager::generateEquCameraInfo(
  const CameraSide& side, std_msgs::Header& header) {
  if (!_camera_ptr.get() &&
      _camera_ptr->modelType() != camodocal::Camera::KANNALA_BRANDT) {
    ROS_ERROR("camera object is error.");
  }
  camodocal::EquidistantCameraPtr equ_ptr =
    boost::dynamic_pointer_cast<camodocal::EquidistantCamera>(_camera_ptr);
  camodocal::EquidistantCamera::Parameters equ_parameters =
    equ_ptr->getParameters();

  sensor_msgs::CameraInfoPtr _camerainfo_ptr =
    boost::make_shared<sensor_msgs::CameraInfo>();

  _camerainfo_ptr->header = header;
  _camerainfo_ptr->distortion_model = toStringCameraInfoName();
  _camerainfo_ptr->height = equ_parameters.imageHeight();
  _camerainfo_ptr->width = equ_parameters.imageWidth();

  _camerainfo_ptr->D.push_back(equ_parameters.k2());
  _camerainfo_ptr->D.push_back(equ_parameters.k3());
  _camerainfo_ptr->D.push_back(equ_parameters.k4());
  _camerainfo_ptr->D.push_back(equ_parameters.k5());

  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  K(0, 0) = equ_parameters.mu();
  K(0, 2) = equ_parameters.u0();
  K(1, 1) = equ_parameters.mv();
  K(1, 2) = equ_parameters.v0();
  _camerainfo_ptr->K[0] = K(0, 0);
  _camerainfo_ptr->K[4] = K(1, 1);
  _camerainfo_ptr->K[2] = K(0, 2);
  _camerainfo_ptr->K[5] = K(1, 2);
  _camerainfo_ptr->K[8] = 1;
#if 0

#else
  Eigen::Matrix4d T = generateRecifyT(side);
  Eigen::Matrix3d rect_R = T.topLeftCorner<3, 3>();
  for (size_t i = 0; i < 3; i++)
    for (size_t j = 0; j < 3; j++)
      _camerainfo_ptr->R.at(i*3 + j) = rect_R(i, j);
  Eigen::Matrix<double, 3, 4> P;
  // P = [K;[0; 0; 0]]*T
  P = (Eigen::Matrix<double, 3, 4>() <<
      K, Eigen::Vector3d::Constant(0)).finished() * T;
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 4; j++) {
      _camerainfo_ptr->P.at(i*4+j) = P(i, j);
    }
  }
#endif
  return _camerainfo_ptr;
}

sensor_msgs::CameraInfoPtr CameraManager::generateCataCameraInfo(
  const CameraSide& side, std_msgs::Header& header) {
  if (!_camera_ptr.get() &&
      _camera_ptr->modelType() != camodocal::Camera::MEI) {
    ROS_ERROR("camera object is error.");
  }

  camodocal::CataCameraPtr cata_ptr =
    boost::dynamic_pointer_cast<camodocal::CataCamera>(_camera_ptr);
  camodocal::CataCamera::Parameters cata_parameters = cata_ptr->getParameters();

  sensor_msgs::CameraInfoPtr _camerainfo_ptr =
    boost::make_shared<sensor_msgs::CameraInfo>();

  _camerainfo_ptr->header = header;
  _camerainfo_ptr->distortion_model = toStringCameraInfoName();
  _camerainfo_ptr->height = cata_parameters.imageHeight();
  _camerainfo_ptr->width = cata_parameters.imageWidth();
  // D[k1, k2, p1, p2, xi]
  _camerainfo_ptr->D.push_back(cata_parameters.k1());
  _camerainfo_ptr->D.push_back(cata_parameters.k2());
  _camerainfo_ptr->D.push_back(cata_parameters.p1());
  _camerainfo_ptr->D.push_back(cata_parameters.p2());
  _camerainfo_ptr->D.push_back(cata_parameters.xi());

  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  K(0, 0) = cata_parameters.gamma1();
  K(0, 2) = cata_parameters.u0();
  K(1, 1) = cata_parameters.gamma2();
  K(1, 2) = cata_parameters.v0();
  _camerainfo_ptr->K[0] = K(0, 0);
  _camerainfo_ptr->K[4] = K(1, 1);
  _camerainfo_ptr->K[2] = K(0, 2);
  _camerainfo_ptr->K[5] = K(1, 2);
  _camerainfo_ptr->K[8] = 1;

  Eigen::Matrix4d T = generateRecifyT(side);
  Eigen::Matrix3d rect_R = T.topLeftCorner<3, 3>();
  for (size_t i = 0; i < 3; i++)
    for (size_t j = 0; j < 3; j++)
      _camerainfo_ptr->R.at(i*3 + j) = rect_R(i, j);
  Eigen::Matrix<double, 3, 4> P;
  // P = [K;[0; 0; 0]]*T
  P = (Eigen::Matrix<double, 3, 4>() <<
    K, Eigen::Vector3d::Constant(0)).finished() * T;
  for (size_t i = 0; i < 3; i++)
    for (size_t j = 0; j < 4; j++)
      _camerainfo_ptr->P.at(i*4+j) = P(i, j);

  return _camerainfo_ptr;
}

sensor_msgs::CameraInfoPtr CameraManager::generatePinholeCameraInfo(
  const CameraSide& side, std_msgs::Header& header) {
  if (!_camera_ptr.get() &&
      _camera_ptr->modelType() != camodocal::Camera::PINHOLE) {
    ROS_ERROR("camera object is error.");
  }

  camodocal::PinholeCameraPtr pin_ptr =
    boost::dynamic_pointer_cast<camodocal::PinholeCamera>(_camera_ptr);
  camodocal::PinholeCamera::Parameters pin_parameters =
    pin_ptr->getParameters();

  sensor_msgs::CameraInfoPtr _camerainfo_ptr =
    boost::make_shared<sensor_msgs::CameraInfo>();

  _camerainfo_ptr->header = header;
  _camerainfo_ptr->distortion_model = toStringCameraInfoName();
  _camerainfo_ptr->height = pin_parameters.imageHeight();
  _camerainfo_ptr->width = pin_parameters.imageWidth();
  _camerainfo_ptr->D.push_back(pin_parameters.k1());
  _camerainfo_ptr->D.push_back(pin_parameters.k2());
  _camerainfo_ptr->D.push_back(pin_parameters.p1());
  _camerainfo_ptr->D.push_back(pin_parameters.p2());

  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  K(0, 0) = pin_parameters.fx();
  K(0, 2) = pin_parameters.cx();
  K(1, 1) = pin_parameters.fy();
  K(1, 2) = pin_parameters.cy();
  _camerainfo_ptr->K[0] = K(0, 0);
  _camerainfo_ptr->K[4] = K(1, 1);
  _camerainfo_ptr->K[2] = K(0, 2);
  _camerainfo_ptr->K[5] = K(1, 2);
  _camerainfo_ptr->K[8] = 1;

  Eigen::Matrix4d T = generateRecifyT(side);
  Eigen::Matrix3d rect_R = T.topLeftCorner<3, 3>();
  for (size_t i = 0; i < 3; i++)
    for (size_t j = 0; j < 3; j++)
      _camerainfo_ptr->R.at(i*3 + j) = rect_R(i, j);
  Eigen::Matrix<double, 3, 4> P;
  // P = [K;[0; 0; 0]]*T
  P = (Eigen::Matrix<double, 3, 4>() <<
    K, Eigen::Vector3d::Constant(0)).finished() * T;
  for (size_t i = 0; i < 3; i++)
    for (size_t j = 0; j < 4; j++)
      _camerainfo_ptr->P.at(i*4+j) = P(i, j);
  return _camerainfo_ptr;
}

Eigen::Matrix4d CameraManager::generateRecifyT(const CameraSide& side) {
  static Eigen::Matrix4d first = loadT(LEFT);
  static Eigen::Matrix4d second = loadT(RIGHT);

  Eigen::Vector3d x = first.topRightCorner<3, 1>() -
    second.topRightCorner<3, 1>();
  Eigen::Vector3d y = first.topLeftCorner<3, 3>().col(2).cross(x);
  Eigen::Vector3d z = x.cross(y);

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() << x.normalized(), y.normalized(), z.normalized();
  if (side == LEFT) {
    return (T.inverse() * first);
  }
  return (T.inverse() * second);
}

Eigen::Matrix4d CameraManager::loadT(const CameraSide& side) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  if (side == LEFT) {
    return T;
  }
  assert(!_extrinsics_yaml.empty() && side == RIGHT);
  cv::FileStorage fs(_extrinsics_yaml, cv::FileStorage::READ);
  double q_x = fs["transform"]["q_x"];
  double q_y = fs["transform"]["q_y"];
  double q_z = fs["transform"]["q_z"];
  double q_w = fs["transform"]["q_w"];
  double t_x = fs["transform"]["t_x"];
  double t_y = fs["transform"]["t_y"];
  double t_z = fs["transform"]["t_z"];

  Eigen::Quaterniond q(q_w, q_x, q_y, q_z);
  q.normalize();
  T.topLeftCorner<3, 3>() = q.toRotationMatrix();
  T.topRightCorner<3, 1>() << t_x, t_y, t_z;
  fs.release();
  return T;
}

Eigen::Matrix3d  CameraManager::solveR(const CameraSide& side) {
  if (_R.find(side) != _R.end()) {
    return _R[side];
  }
  cv::FileStorage fs(_extrinsics_yaml, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "extrinsics_yaml can not open." << std::endl;
  }
  double q_x = fs["transform"]["q_x"];
  double q_y = fs["transform"]["q_y"];
  double q_z = fs["transform"]["q_z"];
  double q_w = fs["transform"]["q_w"];
  double t_x = fs["transform"]["t_x"];
  double t_y = fs["transform"]["t_y"];
  double t_z = fs["transform"]["t_z"];
  fs.release();

  Eigen::Quaterniond q(q_w, q_x, q_y, q_z);
  q.normalize();
  Eigen::Matrix4d H_cam2 = Eigen::Matrix4d::Identity();
  H_cam2.topLeftCorner<3, 3>() = q.toRotationMatrix();
  H_cam2.topRightCorner<3, 1>() << t_x, t_y, t_z;
  Eigen::Matrix4d H_cam1 = Eigen::Matrix4d::Identity();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H_cam1.block<3, 3>(0, 0) +
    H_cam2.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d avgR = svd.matrixU() * svd.matrixV().transpose();
  _R[LEFT] = avgR * H_cam1.block<3, 3>(0, 0).transpose();
  _R[RIGHT] = avgR * H_cam2.block<3, 3>(0, 0).transpose();

  if (side == LEFT) {
    _baseline = 0;
    return _R[LEFT];
  } else {
    _baseline = t_x;
    return _R[RIGHT];
  }
}

std::string CameraManager::toStringCameraInfoName() {
  std::string camera_model_type;
  if (_camera_ptr->modelType() == camodocal::Camera::KANNALA_BRANDT) {
    camera_model_type = "kannala_brandt";
  } else if (_camera_ptr->modelType() == camodocal::Camera::MEI) {
    camera_model_type = "mei";
  } else if (_camera_ptr->modelType() == camodocal::Camera::PINHOLE) {
    camera_model_type = "plumb_bob";
  } else {
    ROS_ERROR("the mode type is not support.");
  }
  return camera_model_type;
}

camodocal::CameraPtr CameraManagerTools::getCamOdoCameraPtr(
  const sensor_msgs::CameraInfoConstPtr& camera_info_ptr) {
  ROS_ASSERT(camera_info_ptr.get() != 0);
  std::string camera_model = camera_info_ptr->distortion_model;
  int w = camera_info_ptr->width;
  int h = camera_info_ptr->height;

  std::vector<double> d(camera_info_ptr->D.begin(), camera_info_ptr->D.end());
  double fx = camera_info_ptr->K[0];
  double fy = camera_info_ptr->K[4];
  double cx = camera_info_ptr->K[2];
  double cy = camera_info_ptr->K[5];

  if (camera_model.compare("kannala_brandt") == 0) {
    // k2, k3, k4, k5
    ROS_ASSERT(d.size() == 4);
    camodocal::EquidistantCameraPtr camera(new camodocal::EquidistantCamera);
    camodocal::EquidistantCamera::Parameters params(
      camera_model, w, h, d[0], d[1], d[2], d[3], fx, fy, cx, cy);
    camera->setParameters(params);
    return camera;
  } else if (camera_model.compare("mei") == 0) {
    // k1, k2, p1, p2, xi
    ROS_ASSERT(d.size() == 5);
    camodocal::CataCameraPtr camera(new camodocal::CataCamera);
    camodocal::CataCamera::Parameters params(
      camera_model, w, h, d[4], d[0], d[1], d[2], d[3], fx, fy, cx, cy);
    camera->setParameters(params);
    return camera;
  } else if (camera_model.compare("plumb_bob") == 0) {
    // k1, k2, p1, p2
    ROS_ASSERT(d.size() == 4);
    camodocal::PinholeCameraPtr camera(new camodocal::PinholeCamera);
    camodocal::PinholeCamera::Parameters params(
      camera_model, w, h, d[0], d[1], d[2], d[3], fx, fy, cx, cy);
    camera->setParameters(params);
    return camera;
  }
  return camodocal::CameraPtr();
}

sensor_msgs::CameraInfoPtr CameraManagerTools::generateCameraFromYamlFile(
  const std::string& file_name) {
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    return sensor_msgs::CameraInfoPtr();
  }

  camodocal::Camera::ModelType modelType = camodocal::Camera::KANNALA_BRANDT;
  if (!fs["model_type"].isNone()) {
    std::string sModelType;
    fs["model_type"] >> sModelType;

    if (boost::iequals(sModelType, "KANNALA_BRANDT")) {
      modelType = camodocal::Camera::KANNALA_BRANDT;
    } else if (boost::iequals(sModelType, "MEI")) {
      modelType = camodocal::Camera::MEI;
    } else if (boost::iequals(sModelType, "PINHOLE")) {
      modelType = camodocal::Camera::PINHOLE;
    } else {
      ROS_ERROR("# ERROR: Unkonw camera model");
    }
  }
  switch (modelType) {
    case camodocal::Camera::KANNALA_BRANDT:
    {
      camodocal::EquidistantCameraPtr camera(new camodocal::EquidistantCamera);
      camodocal::EquidistantCamera::Parameters params = camera->getParameters();
      params.readFromYamlFile(file_name);
      camera->setParameters(params);
      //  odo_cam_ptr = camera;
      return generatorCameraInfo(params);
    }
    case camodocal::Camera::PINHOLE:
    {
      camodocal::PinholeCameraPtr camera(new camodocal::PinholeCamera);
      camodocal::PinholeCamera::Parameters params = camera->getParameters();
      params.readFromYamlFile(file_name);
      camera->setParameters(params);
      // odo_cam_ptr = camera;
      return generatorCameraInfo(params);
    }
    case camodocal::Camera::MEI:
    {
      camodocal::CataCameraPtr camera(new camodocal::CataCamera);
      camodocal::CataCamera::Parameters params = camera->getParameters();
      params.readFromYamlFile(file_name);
      camera->setParameters(params);
      // odo_cam_ptr = camera;
      return generatorCameraInfo(params);
    }
    default:
    {
      camodocal::EquidistantCameraPtr camera(new camodocal::EquidistantCamera);
      camodocal::EquidistantCamera::Parameters params = camera->getParameters();
      params.readFromYamlFile(file_name);
      camera->setParameters(params);
      // odo_cam_ptr = camera;
      return generatorCameraInfo(params);
    }
  }

  return sensor_msgs::CameraInfoPtr();
}

sensor_msgs::CameraInfoPtr CameraManagerTools::generatorCameraInfo(
  camodocal::EquidistantCamera::Parameters& params) {
  sensor_msgs::CameraInfoPtr _camerainfo_ptr =
    boost::make_shared<sensor_msgs::CameraInfo>();
  _camerainfo_ptr->distortion_model = "kannala_brandt";
  _camerainfo_ptr->height = params.imageHeight();
  _camerainfo_ptr->width = params.imageWidth();

  _camerainfo_ptr->D.push_back(params.k2());
  _camerainfo_ptr->D.push_back(params.k3());
  _camerainfo_ptr->D.push_back(params.k4());
  _camerainfo_ptr->D.push_back(params.k5());

  _camerainfo_ptr->K[0] = params.mu();
  _camerainfo_ptr->K[4] = params.mv();
  _camerainfo_ptr->K[2] = params.u0();
  _camerainfo_ptr->K[5] = params.v0();
  _camerainfo_ptr->K[8] = 1;

  return _camerainfo_ptr;
}
sensor_msgs::CameraInfoPtr CameraManagerTools::generatorCameraInfo(
  camodocal::CataCamera::Parameters& params) {
  sensor_msgs::CameraInfoPtr _camerainfo_ptr =
    boost::make_shared<sensor_msgs::CameraInfo>();
  _camerainfo_ptr->distortion_model = "mei";
  _camerainfo_ptr->height = params.imageHeight();
  _camerainfo_ptr->width = params.imageWidth();

  // D[k1, k2, p1, p2, xi]
  _camerainfo_ptr->D.push_back(params.k1());
  _camerainfo_ptr->D.push_back(params.k2());
  _camerainfo_ptr->D.push_back(params.p1());
  _camerainfo_ptr->D.push_back(params.p2());
  _camerainfo_ptr->D.push_back(params.xi());

  _camerainfo_ptr->K[0] = params.gamma1();
  _camerainfo_ptr->K[4] = params.gamma2();
  _camerainfo_ptr->K[2] = params.u0();
  _camerainfo_ptr->K[5] = params.v0();
  _camerainfo_ptr->K[8] = 1;
  return _camerainfo_ptr;
}
sensor_msgs::CameraInfoPtr CameraManagerTools::generatorCameraInfo(
  camodocal::PinholeCamera::Parameters& params) {
  sensor_msgs::CameraInfoPtr _camerainfo_ptr =
    boost::make_shared<sensor_msgs::CameraInfo>();
  _camerainfo_ptr->distortion_model = "plumb_bob";

  _camerainfo_ptr->height = params.imageHeight();
  _camerainfo_ptr->width = params.imageWidth();

  // D[k1, k2, p1, p2]
  _camerainfo_ptr->D.push_back(params.k1());
  _camerainfo_ptr->D.push_back(params.k2());
  _camerainfo_ptr->D.push_back(params.p1());
  _camerainfo_ptr->D.push_back(params.p2());

  _camerainfo_ptr->K[0] = params.fx();
  _camerainfo_ptr->K[4] = params.fy();
  _camerainfo_ptr->K[2] = params.cx();
  _camerainfo_ptr->K[5] = params.cy();
  _camerainfo_ptr->K[8] = 1;
  return _camerainfo_ptr;
}

sensor_msgs::CameraInfoPtr CameraManagerTools::setDistortionToZereo(
  const sensor_msgs::CameraInfoConstPtr& camera_info_ptr) {
  sensor_msgs::CameraInfoPtr target_camera_info_ptr =
    boost::make_shared<sensor_msgs::CameraInfo>(*camera_info_ptr);
  for (auto& elem : target_camera_info_ptr->D) {
    elem = 0;
  }
  return target_camera_info_ptr;
}
}  // namespace mynteye_image_pipeline
