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
#include "device_writer.h"

#include <vector>

#include <opencv2/core/core.hpp>

#include "mynteye/logger.h"
#include "mynteye/util/files.h"

#define SAVE_LATEST_VERSION Version(1, 2)

MYNTEYE_BEGIN_NAMESPACE

namespace tools {

DeviceWriter::DeviceWriter(std::shared_ptr<Device> device) : device_(device) {
  VLOG(2) << __func__;
}

DeviceWriter::~DeviceWriter() {
  VLOG(2) << __func__;
}

bool DeviceWriter::WriteDeviceInfo(const dev_info_t &info) {
  // Update device info
  auto &&dev_info = device_->GetInfo();
  dev_info->lens_type = Type(info.lens_type);
  dev_info->imu_type = Type(info.imu_type);
  dev_info->nominal_baseline = info.nominal_baseline;

  if (device_->SetFiles(dev_info.get(), nullptr, nullptr)) {
    LOG(INFO) << "Write device info success";
    LOG(INFO) << "Device info: {name: " << dev_info->name
              << ", serial_number: " << dev_info->serial_number
              << ", firmware_version: "
              << dev_info->firmware_version.to_string()
              << ", hardware_version: "
              << dev_info->hardware_version.to_string()
              << ", spec_version: " << dev_info->spec_version.to_string()
              << ", lens_type: " << dev_info->lens_type.to_string()
              << ", imu_type: " << dev_info->imu_type.to_string()
              << ", nominal_baseline: " << dev_info->nominal_baseline
              << ", auxiliary_chip_version: "
              << dev_info->auxiliary_chip_version.to_string()
              << ", isp_version: "
              << dev_info->isp_version.to_string()<< "}";
    return true;
  } else {
    LOG(ERROR) << "Write device info failed";
    return false;
  }
}

bool DeviceWriter::WriteDeviceInfo(const std::string &filepath) {
  return WriteDeviceInfo(LoadDeviceInfo(filepath));
}

bool DeviceWriter::WriteImgParams(const img_params_map_t &img_params_map) {
  img_params_map_t *img_params_new =
      const_cast<img_params_map_t *>(&img_params_map);
  // Update image params with raw
  auto &&img_params_raw = device_->GetImgParams();
  for (auto entry_raw : img_params_raw) {
    // Add raw params if not load this resolution
    if (img_params_new->find(entry_raw.first) == img_params_new->end()) {
      (*img_params_new)[entry_raw.first] = entry_raw.second;
    }
  }

  if (device_->SetFiles(nullptr, img_params_new, nullptr)) {
    LOG(INFO) << "Write img params success";
    for (auto it = img_params_new->begin(); it != img_params_new->end(); it++) {
      LOG(INFO) << "Resolution: {width: " << (*it).first.width
                << ", height: " << (*it).first.height << "}";
      LOG(INFO) << "Intrinsics left: {" << *(*it).second.in_left << "}";
      LOG(INFO) << "Intrinsics right: {" << *(*it).second.in_right << "}";
      LOG(INFO) << "Extrinsics right to left: {"
                << (*it).second.ex_right_to_left << "}";
    }
    return true;
  } else {
    LOG(ERROR) << "Write img params failed";
    return false;
  }
}

bool DeviceWriter::WriteImgParams(const std::string &filepath) {
  return WriteImgParams(LoadImgParams(filepath));
}

bool DeviceWriter::WriteImuParams(const imu_params_t &params) {
  if (device_->SetFiles(
          nullptr, nullptr, const_cast<imu_params_t *>(&params))) {
    LOG(INFO) << "Write imu params success";
    LOG(INFO) << "Imu intrinsics accel: {" << params.in_accel << "}";
    LOG(INFO) << "Imu intrinsics gyro: {" << params.in_gyro << "}";
    LOG(INFO) << "Imu extrinsics left to imu: {" << params.ex_left_to_imu
              << "}";
    return true;
  } else {
    LOG(ERROR) << "Write imu params failed";
    return false;
  }
}

bool DeviceWriter::WriteImuParams(const std::string &filepath) {
  return WriteImuParams(LoadImuParams(filepath));
}

namespace {

cv::FileStorage &operator<<(cv::FileStorage &fs, const IntrinsicsPinhole &in) {
  fs << "{"
     << "fx" << in.fx << "fy" << in.fy
     << "cx" << in.cx << "cy" << in.cy
     << "coeffs" << std::vector<double>(in.coeffs, in.coeffs + 5) << "}";
  return fs;
}

cv::FileStorage &operator<<(cv::FileStorage &fs,
    const IntrinsicsEquidistant &in) {
  fs << "{"
     << "coeffs" << std::vector<double>(in.coeffs, in.coeffs + 8) << "}";
  return fs;
}

cv::FileStorage &operator<<(cv::FileStorage &fs,
    const std::shared_ptr<IntrinsicsBase> &in) {
  switch (in->calib_model()) {
    case CalibrationModel::PINHOLE:
      return fs << *std::dynamic_pointer_cast<IntrinsicsPinhole>(in);
    case CalibrationModel::KANNALA_BRANDT:
      return fs << *std::dynamic_pointer_cast<IntrinsicsEquidistant>(in);
    default:
      LOG(FATAL) << "Unknown calib model: " << in->calib_model();
      return fs;
  }
}

cv::FileStorage &operator<<(cv::FileStorage &fs, const ImuIntrinsics &in) {
  std::vector<double> scales;
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      scales.push_back(in.scale[i][j]);
    }
  }
  std::vector<double> assembly;
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      assembly.push_back(in.assembly[i][j]);
    }
  }
  fs << "{"
     << "scale" << scales << "assembly" << assembly << "drift"
     << std::vector<double>(in.drift, in.drift + 3) << "noise"
     << std::vector<double>(in.noise, in.noise + 3) << "bias"
     << std::vector<double>(in.bias, in.bias + 3) << "x"
     << std::vector<double>(in.x, in.x + 2) << "y"
     << std::vector<double>(in.y, in.y + 2) << "z"
     << std::vector<double>(in.z, in.z + 2) << "}";
  return fs;
}

cv::FileStorage &operator<<(cv::FileStorage &fs, const Extrinsics &ex) {
  std::vector<double> rotations;
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      rotations.push_back(ex.rotation[i][j]);
    }
  }
  fs << "{"
     << "rotation" << rotations << "translation"
     << std::vector<double>(ex.translation, ex.translation + 3) << "}";
  return fs;
}

cv::FileStorage &operator<<(
    cv::FileStorage &fs, const device::img_params_t &params) {
  fs << "{"
     << "model" << static_cast<std::uint8_t>(params.in_left->calib_model())
     << "width" << params.in_left->width
     << "height" << params.in_left->height
     << "in_left" << params.in_left
     << "in_right" << params.in_right
     << "ex_right_to_left" << params.ex_right_to_left << "}";
  return fs;
}

cv::FileStorage &operator<<(
    cv::FileStorage &fs, const DeviceWriter::img_params_map_t &img_params_map) {
  fs << "[";
  std::map<Resolution, device::img_params_t>::const_iterator it;
  for (it = img_params_map.begin(); it != img_params_map.end(); it++) {
    fs << (*it).second;
  }
  fs << "]";
  return fs;
}

}  // namespace

bool DeviceWriter::SaveDeviceInfo(
    const dev_info_t &info, const std::string &filepath) {
  using FileStorage = cv::FileStorage;
  FileStorage fs(filepath, FileStorage::WRITE);
  if (!fs.isOpened()) {
    LOG(ERROR) << "Failed to save file: " << filepath;
    return false;
  }
  fs << "device_name" << info.name;
  fs << "serial_number" << info.serial_number;
  fs << "firmware_version" << info.firmware_version.to_string();
  fs << "hardware_version" << info.hardware_version.to_string();
  fs << "spec_version" << info.spec_version.to_string();
  fs << "lens_type" << info.lens_type.to_string();
  fs << "imu_type" << info.imu_type.to_string();
  fs << "nominal_baseline" << info.nominal_baseline;
  fs << "auxiliary_chip_version" << info.auxiliary_chip_version.to_string();
  fs << "isp_version" << info.isp_version.to_string();
  // save other infos according to spec_version
  fs.release();
  return true;
}

bool DeviceWriter::SaveImgParams(
    const img_params_map_t &img_params_map,
    const std::string &filepath) {
  if (img_params_map.empty()) {
    return false;
  }
  std::string version = img_params_map.begin()->second.version;
  if (Version(version) > SAVE_LATEST_VERSION) {
    LOG(ERROR) << "Failed to save img params of version " << version
        << ", please use latest SDK.";
    return false;
  }

  // always save img params with latest version format
  using FileStorage = cv::FileStorage;
  FileStorage fs(filepath, FileStorage::WRITE);
  if (!fs.isOpened()) {
    LOG(ERROR) << "Failed to save file: " << filepath;
    return false;
  }
  fs << "version" << SAVE_LATEST_VERSION.to_string()
     << "img_params" << img_params_map;
  fs.release();
  return true;
}

bool DeviceWriter::SaveImuParams(
    const imu_params_t &params, const std::string &filepath) {
  if (!params.ok) return false;
  std::string version = params.version;
  if (Version(version) > SAVE_LATEST_VERSION) {
    LOG(ERROR) << "Failed to save imu params of version " << version
        << ", please use latest SDK.";
    return false;
  }

  // always save imu params with latest version format
  using FileStorage = cv::FileStorage;
  FileStorage fs(filepath, FileStorage::WRITE);
  if (!fs.isOpened()) {
    LOG(ERROR) << "Failed to save file: " << filepath;
    return false;
  }
  fs << "version" << SAVE_LATEST_VERSION.to_string()
     << "in_accel" << params.in_accel
     << "in_gyro" << params.in_gyro
     << "ex_left_to_imu" << params.ex_left_to_imu;
  fs.release();
  return true;
}

void DeviceWriter::SaveAllInfos(const std::string &dir) {
  if (!files::mkdir(dir)) {
    LOG(FATAL) << "Create directory failed: " << dir;
  }
  SaveDeviceInfo(*device_->GetInfo(), dir + MYNTEYE_OS_SEP "device.info");
  SaveImgParams(device_->GetImgParams(), dir + MYNTEYE_OS_SEP "img.params");
  SaveImuParams(device_->GetImuParams(), dir + MYNTEYE_OS_SEP "imu.params");
}

namespace {

// old
void to_intrinsics(
    const std::uint16_t &width, const std::uint16_t &height,
    const std::uint8_t &model, const cv::Mat &M, const cv::Mat &D,
    IntrinsicsPinhole *in) {
  in->width = width;
  in->height = height;
  /*
  fx,  0, cx,
   0, fy, cy,
   0,  0,  1
  */
  in->fx = M.at<double>(0, 0);
  in->fy = M.at<double>(1, 1);
  in->cx = M.at<double>(0, 2);
  in->cy = M.at<double>(1, 2);
  /* k1, k2, p1, p2, k3 */
  in->model = model;
  LOG_IF(FATAL, D.cols < 5) << "Distortion coefficients must >= 5 columns";
  for (std::size_t i = 0; i < 5; i++) {
    in->coeffs[i] = D.at<double>(i);
  }
}

// old
void to_extrinsics(const cv::Mat &R, const cv::Mat &T, Extrinsics *ex) {
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      ex->rotation[i][j] = R.at<double>(i, j);
    }
  }
  for (std::size_t i = 0; i < 3; i++) {
    ex->translation[i] = T.at<double>(i);
  }
}
// old
void operator>>(const cv::FileNode &n, IntrinsicsPinhole &in) {
  n["width"] >> in.width;
  n["height"] >> in.height;
  n["fx"] >> in.fx;
  n["fy"] >> in.fy;
  n["cx"] >> in.cx;
  n["cy"] >> in.cy;
  n["model"] >> in.model;
  for (std::size_t i = 0; i < 5; i++) {
    in.coeffs[i] = n["coeffs"][i];
  }
}

void operator>>(const cv::FileNode &n, ImuIntrinsics &in) {
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      in.scale[i][j] = n["scale"][3 * i + j];
    }
  }
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      in.assembly[i][j] = n["assembly"][3 * i + j];
    }
  }
  for (std::size_t i = 0; i < 3; i++) {
    in.drift[i] = n["drift"][i];
  }
  for (std::size_t i = 0; i < 3; i++) {
    in.noise[i] = n["noise"][i];
  }
  for (std::size_t i = 0; i < 3; i++) {
    in.bias[i] = n["bias"][i];
  }
  for (std::size_t i = 0; i < 2; i++) {
    in.x[i] = n["x"][i];
  }
  for (std::size_t i = 0; i < 2; i++) {
    in.y[i] = n["y"][i];
  }
  for (std::size_t i = 0; i < 2; i++) {
    in.z[i] = n["z"][i];
  }
}

void operator>>(const cv::FileNode &n, Extrinsics &ex) {
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      ex.rotation[i][j] = n["rotation"][3 * i + j];
    }
  }
  for (std::size_t i = 0; i < 3; i++) {
    ex.translation[i] = n["translation"][i];
  }
}

// old
void operator>>(const cv::FileNode &n, DeviceWriter::img_params_t &params) {
  auto in_left = std::make_shared<IntrinsicsPinhole>();
  auto in_right = std::make_shared<IntrinsicsPinhole>();
  params.in_left = in_left;
  params.in_right = in_right;
  n["in_left"] >> *in_left;
  n["in_right"] >> *in_right;
  n["ex_right_to_left"] >> params.ex_right_to_left;
}

std::shared_ptr<IntrinsicsPinhole> to_intrinsics_pinhole(
    const cv::FileNode &n, const std::uint8_t &model,
    const std::uint16_t &width, const std::uint16_t &height) {
  auto in = std::make_shared<IntrinsicsPinhole>();
  in->width = width;
  in->height = height;
  in->model = model;
  n["fx"] >> in->fx;
  n["fy"] >> in->fy;
  n["cx"] >> in->cx;
  n["cy"] >> in->cy;
  for (std::size_t i = 0; i < 5; i++) {
    in->coeffs[i] = n["coeffs"][i];
  }
  return in;
}

std::shared_ptr<IntrinsicsEquidistant> to_intrinsics_equidistant(
    const cv::FileNode &n, const std::uint8_t &model,
    const std::uint16_t &width, const std::uint16_t &height) {
  auto in = std::make_shared<IntrinsicsEquidistant>();
  in->width = width;
  in->height = height;
  for (std::size_t i = 0; i < 8; i++) {
    in->coeffs[i] = n["coeffs"][i];
  }
  MYNTEYE_UNUSED(model)
  return in;
}

DeviceWriter::img_params_t to_img_params(
    const cv::FileNode &n, const std::uint8_t &model,
    const std::uint16_t &width, const std::uint16_t &height) {
  DeviceWriter::img_params_t params;
  params.ok = false;

  CalibrationModel calib_model = static_cast<CalibrationModel>(model);
  switch (calib_model) {
    case CalibrationModel::PINHOLE: {
      params.ok = true;
      params.in_left = to_intrinsics_pinhole(
          n["in_left"], model, width, height);
      params.in_right = to_intrinsics_pinhole(
          n["in_right"], model, width, height);
      n["ex_right_to_left"] >> params.ex_right_to_left;
    } break;
    case CalibrationModel::KANNALA_BRANDT: {
      params.ok = true;
      params.in_left = to_intrinsics_equidistant(
          n["in_left"], model, width, height);
      params.in_right = to_intrinsics_equidistant(
          n["in_right"], model, width, height);
      n["ex_right_to_left"] >> params.ex_right_to_left;
    } break;
    default:
      LOG(FATAL) << "Could not load img params as unknown calib model"
          ", please use latest SDK.";
  }
  return params;
}

}  // namespace

DeviceWriter::dev_info_t DeviceWriter::LoadDeviceInfo(
    const std::string &filepath) {
  using FileStorage = cv::FileStorage;
  FileStorage fs(filepath, FileStorage::READ);
  if (!fs.isOpened()) {
    LOG(FATAL) << "Failed to load file: " << filepath;
  }
  DeviceInfo info;
  info.lens_type = Type(std::string(fs["lens_type"]));
  info.imu_type = Type(std::string(fs["imu_type"]));
  fs["nominal_baseline"] >> info.nominal_baseline;
  fs.release();
  return info;
}

DeviceWriter::img_params_map_t DeviceWriter::LoadImgParams(
    const std::string &filepath) {
  using FileStorage = cv::FileStorage;
  FileStorage fs(filepath, FileStorage::READ);
  if (!fs.isOpened()) {
    LOG(FATAL) << "Failed to load file: " << filepath;
  }

  img_params_map_t img_params_map;
  if (!fs["version"].isNone()) {
    std::string version = std::string(fs["version"]);
    // load params according to verison
    if (version == "1.0") {
      fs["img_params_map"][0] >> img_params_map[{752, 480}];
    } else if (version == "1.1") {
      fs["img_params_map"][0] >> img_params_map[{1280, 400}];
      fs["img_params_map"][1] >> img_params_map[{2560, 800}];
    } else if (version == "1.2") {
      auto node = fs["img_params"];
      for (auto it = node.begin(); it < node.end(); it++) {
        std::uint8_t model;
        std::uint16_t width, height;
        (*it)["model"] >> model;
        (*it)["width"] >> width;
        (*it)["height"] >> height;
        auto params = to_img_params(*it, model, width, height);
        if (params.ok) {
          params.version = version;
          img_params_map[{width, height}] = params;
        }
      }
    } else {
      LOG(ERROR) << "Failed to load img params of version " << version
          << ", please use latest SDK.";
    }
  } else {
    // load old params s1030
    auto in_left = std::make_shared<IntrinsicsPinhole>();
    auto in_right = std::make_shared<IntrinsicsPinhole>();
    Extrinsics ex_right_to_left;
    if (fs["in_left"].isNone()) {
      std::uint16_t w = 752;
      std::uint16_t h = 480;
      std::uint8_t m = 0;
      if (!fs["width"].isNone())
        w = static_cast<int>(fs["width"]);
      if (!fs["height"].isNone())
        h = static_cast<int>(fs["height"]);
      if (!fs["model"].isNone())
        m = static_cast<int>(fs["model"]);

      cv::Mat M1, D1, M2, D2, R, T;
      fs["M1"] >> M1;
      fs["D1"] >> D1;
      fs["M2"] >> M2;
      fs["D2"] >> D2;
      fs["R"] >> R;
      fs["T"] >> T;

      to_intrinsics(w, h, m, M1, D1, in_left.get());
      to_intrinsics(w, h, m, M2, D2, in_right.get());
      to_extrinsics(R, T, &ex_right_to_left);
    } else {
      fs["in_left"] >> *in_left;
      fs["in_right"] >> *in_right;
      fs["ex_right_to_left"] >> ex_right_to_left;
    }
    img_params_map[{752, 480}] = {
      true, "1.0", in_left, in_right, ex_right_to_left
    };
  }

  fs.release();
  return img_params_map;
}

DeviceWriter::imu_params_t DeviceWriter::LoadImuParams(
    const std::string &filepath) {
  using FileStorage = cv::FileStorage;
  FileStorage fs(filepath, FileStorage::READ);
  if (!fs.isOpened()) {
    LOG(FATAL) << "Failed to load file: " << filepath;
  }

  imu_params_t params;
  if (!fs["version"].isNone()) {
    std::string version = std::string(fs["version"]);
    // load params according to verison
    if (version == "1.2") {
      params.version = version;
      fs["in_accel"] >> params.in_accel;
      fs["in_gyro"] >> params.in_gyro;
      fs["ex_left_to_imu"] >> params.ex_left_to_imu;
    } else {
      LOG(ERROR) << "Failed to load imu params of version " << version
          << ", please use latest SDK.";
    }
  } else {
    // load old params
    fs["in_accel"] >> params.in_accel;
    fs["in_gyro"] >> params.in_gyro;
    fs["ex_left_to_imu"] >> params.ex_left_to_imu;
  }

  fs.release();
  return params;
}

}  // namespace tools

MYNTEYE_END_NAMESPACE
