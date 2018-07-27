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
#include "writer/device_writer.h"

#include <opencv2/core/core.hpp>

#include <glog/logging.h>

#include <vector>

#include "mynteye/device.h"
#include "mynteye/files.h"

MYNTEYE_BEGIN_NAMESPACE

namespace tools {

DeviceWriter::DeviceWriter(std::shared_ptr<Device> device) : device_(device) {
  VLOG(2) << __func__;
}

DeviceWriter::~DeviceWriter() {
  VLOG(2) << __func__;
}

bool DeviceWriter::WriteDeviceInfo(const dev_info_t &info) {
  auto &&channels = device_->channels();
  auto &&dev_info = device_->GetInfo();
  dev_info->lens_type = Type(info.lens_type);
  dev_info->imu_type = Type(info.imu_type);
  dev_info->nominal_baseline = info.nominal_baseline;
  if (channels->SetFiles(dev_info.get(), nullptr, nullptr)) {
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
              << ", nominal_baseline: " << dev_info->nominal_baseline << "}";
    return true;
  } else {
    LOG(ERROR) << "Write device info failed";
    return false;
  }
}

bool DeviceWriter::WriteDeviceInfo(const std::string &filepath) {
  return WriteDeviceInfo(LoadDeviceInfo(filepath));
}

bool DeviceWriter::WriteImgParams(const img_params_t &params) {
  auto &&channels = device_->channels();
  auto &&dev_info = device_->GetInfo();
  if (channels->SetFiles(
          nullptr, const_cast<img_params_t *>(&params), nullptr,
          &dev_info->spec_version)) {
    LOG(INFO) << "Write img params success";
    LOG(INFO) << "Intrinsics left: {" << params.in_left << "}";
    LOG(INFO) << "Intrinsics right: {" << params.in_right << "}";
    LOG(INFO) << "Extrinsics right to left: {" << params.ex_right_to_left
              << "}";
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
  auto &&channels = device_->channels();
  auto &&dev_info = device_->GetInfo();
  if (channels->SetFiles(
          nullptr, nullptr, const_cast<imu_params_t *>(&params),
          &dev_info->spec_version)) {
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

cv::FileStorage &operator<<(cv::FileStorage &fs, const Intrinsics &in) {
  fs << "{"
     << "width" << in.width << "height" << in.height << "fx" << in.fx << "fy"
     << in.fy << "cx" << in.cx << "cy" << in.cy << "model" << in.model
     << "coeffs" << std::vector<double>(in.coeffs, in.coeffs + 5) << "}";
  return fs;
}

cv::FileStorage &operator<<(
    cv::FileStorage &fs, const std::vector<Intrinsics> &vec) {
  fs << "[";
  for (auto &&in : vec)
    fs << in;
  fs << "]";
  return fs;
}

cv::FileStorage &operator<<(cv::FileStorage &fs, const ImuIntrinsics &in) {
  std::vector<double> scales;
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      scales.push_back(in.scale[i][j]);
    }
  }
  fs << "{"
     << "scale" << scales << "drift"
     << std::vector<double>(in.drift, in.drift + 3) << "noise"
     << std::vector<double>(in.noise, in.noise + 3) << "bias"
     << std::vector<double>(in.bias, in.bias + 3) << "}";
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
  fs.release();
  return true;
}

bool DeviceWriter::SaveImgParams(
    const img_params_t &params, const std::string &filepath) {
  using FileStorage = cv::FileStorage;
  FileStorage fs(filepath, FileStorage::WRITE);
  if (!fs.isOpened()) {
    LOG(ERROR) << "Failed to save file: " << filepath;
    return false;
  }
  fs << "in_left" << std::vector<Intrinsics>{params.in_left} << "in_right"
     << std::vector<Intrinsics>{params.in_right} << "ex_right_to_left"
     << params.ex_right_to_left;
  fs.release();
  return true;
}

bool DeviceWriter::SaveImuParams(
    const imu_params_t &params, const std::string &filepath) {
  using FileStorage = cv::FileStorage;
  FileStorage fs(filepath, FileStorage::WRITE);
  if (!fs.isOpened()) {
    LOG(ERROR) << "Failed to save file: " << filepath;
    return false;
  }
  fs << "in_accel" << params.in_accel << "in_gyro" << params.in_gyro
     << "ex_left_to_imu" << params.ex_left_to_imu;
  fs.release();
  return true;
}

void DeviceWriter::SaveAllInfos(const std::string &dir) {
  if (!files::mkdir(dir)) {
    LOG(FATAL) << "Create directory failed: " << dir;
  }
  SaveDeviceInfo(*device_->GetInfo(), dir + OS_SEP "device.info");
  SaveImgParams(
      {false, device_->GetIntrinsics(Stream::LEFT),
       device_->GetIntrinsics(Stream::RIGHT),
       device_->GetExtrinsics(Stream::RIGHT, Stream::LEFT)},
      dir + OS_SEP "img.params");
  auto &&m_in = device_->GetMotionIntrinsics();
  SaveImuParams(
      {
          false, m_in.accel, m_in.gyro,
          device_->GetMotionExtrinsics(Stream::LEFT),
      },
      dir + OS_SEP "imu.params");
}

namespace {

void to_intrinsics(
    const std::uint16_t &width, const std::uint16_t &height,
    const std::uint8_t &model, const cv::Mat &M, const cv::Mat &D,
    Intrinsics *in) {
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

void operator>>(const cv::FileNode &n, Intrinsics &in) {
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
    in.drift[i] = n["drift"][i];
  }
  for (std::size_t i = 0; i < 3; i++) {
    in.noise[i] = n["noise"][i];
  }
  for (std::size_t i = 0; i < 3; i++) {
    in.bias[i] = n["bias"][i];
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

DeviceWriter::img_params_t DeviceWriter::LoadImgParams(
    const std::string &filepath) {
  using FileStorage = cv::FileStorage;
  FileStorage fs(filepath, FileStorage::READ);
  if (!fs.isOpened()) {
    LOG(FATAL) << "Failed to load file: " << filepath;
  }

  img_params_t params;
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

    to_intrinsics(w, h, m, M1, D1, &params.in_left);
    to_intrinsics(w, h, m, M2, D2, &params.in_right);
    to_extrinsics(R, T, &params.ex_right_to_left);
  } else {
    fs["in_left"][0] >> params.in_left;
    fs["in_right"][0] >> params.in_right;
    fs["ex_right_to_left"] >> params.ex_right_to_left;
  }

  fs.release();
  return params;
}

DeviceWriter::imu_params_t DeviceWriter::LoadImuParams(
    const std::string &filepath) {
  using FileStorage = cv::FileStorage;
  FileStorage fs(filepath, FileStorage::READ);
  if (!fs.isOpened()) {
    LOG(FATAL) << "Failed to load file: " << filepath;
  }
  imu_params_t params;
  fs["in_accel"] >> params.in_accel;
  fs["in_gyro"] >> params.in_gyro;
  fs["ex_left_to_imu"] >> params.ex_left_to_imu;
  fs.release();
  return params;
}

}  // namespace tools

MYNTEYE_END_NAMESPACE
