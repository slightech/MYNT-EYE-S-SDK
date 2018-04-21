#ifndef MYNTEYE_TOOLS_DEVICE_WRITER_H_  // NOLINT
#define MYNTEYE_TOOLS_DEVICE_WRITER_H_
#pragma once

#include <memory>
#include <string>

#include "mynteye/mynteye.h"

#include "internal/channels.h"
#include "internal/types.h"

MYNTEYE_BEGIN_NAMESPACE

class Device;

namespace tools {

class DeviceWriter {
 public:
  using dev_info_t = DeviceInfo;
  using img_params_t = Channels::img_params_t;
  using imu_params_t = Channels::imu_params_t;

  explicit DeviceWriter(std::shared_ptr<Device> device);
  ~DeviceWriter();

  bool WriteDeviceInfo(const dev_info_t &info);
  bool WriteDeviceInfo(const std::string &filepath);

  bool WriteImgParams(const img_params_t &params);
  bool WriteImgParams(const std::string &filepath);

  bool WriteImuParams(const imu_params_t &params);
  bool WriteImuParams(const std::string &filepath);

  bool SaveDeviceInfo(const dev_info_t &info, const std::string &filepath);
  bool SaveImgParams(const img_params_t &params, const std::string &filepath);
  bool SaveImuParams(const imu_params_t &params, const std::string &filepath);

  /** Save all infos of this device */
  void SaveAllInfos(const std::string &dir);

 private:
  dev_info_t LoadDeviceInfo(const std::string &filepath);
  img_params_t LoadImgParams(const std::string &filepath);
  imu_params_t LoadImuParams(const std::string &filepath);

  std::shared_ptr<Device> device_;
};

}  // namespace tools

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_TOOLS_DEVICE_WRITER_H_ NOLINT
