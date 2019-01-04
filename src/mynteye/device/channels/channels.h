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
#ifndef MYNTEYE_DEVICE_CHANNELS_CHANNELS_H_
#define MYNTEYE_DEVICE_CHANNELS_CHANNELS_H_
#pragma once

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "mynteye/mynteye.h"
#include "mynteye/types.h"
#include "mynteye/device/device.h"
#include "mynteye/device/types.h"
#include "mynteye/uvc/uvc.h"

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

struct device;
struct xu;

}  // namespace uvc

class ChannelsAdapter;

class MYNTEYE_API Channels {
 public:
  typedef enum Channel {
    CHANNEL_CAM_CTRL = 1,
    CHANNEL_HALF_DUPLEX = 2,
    CHANNEL_IMU_WRITE = 3,
    CHANNEL_IMU_READ = 4,
    CHANNEL_FILE = 5,
    CHANNEL_LAST
  } channel_t;

  typedef struct ControlInfo {
    std::int32_t min;
    std::int32_t max;
    std::int32_t def;
  } control_info_t;

  typedef enum XuCmd {
    XU_CMD_ZDC = 0xE6,    // zero drift calibration
    XU_CMD_ERASE = 0xDE,  // erase chip
    XU_CMD_LAST
  } xu_cmd_t;

  typedef enum FileId {
    FID_DEVICE_INFO = 1,  // device info
    FID_IMG_PARAMS = 2,   // image intrinsics & extrinsics
    FID_IMU_PARAMS = 4,   // imu intrinsics & extrinsics
    FID_LAST,
  } file_id_t;

  using imu_callback_t = std::function<void(const ImuPacket &packet)>;

  using device_info_t = DeviceInfo;

  using img_params_t = std::map<Resolution, device::img_params_t>;
  using imu_params_t = device::imu_params_t;

  Channels(const std::shared_ptr<uvc::device> &device,
           const std::shared_ptr<ChannelsAdapter> &adapter);
  ~Channels();

  std::int32_t GetAccelRangeDefault();
  std::int32_t GetGyroRangeDefault();

  void LogControlInfos() const;
  void UpdateControlInfos();
  control_info_t GetControlInfo(const Option &option) const;

  std::int32_t GetControlValue(const Option &option) const;
  void SetControlValue(const Option &option, std::int32_t value);

  bool RunControlAction(const Option &option) const;

  void SetImuCallback(imu_callback_t callback);
  void DoImuTrack();

  void StartImuTracking(imu_callback_t callback = nullptr);
  void StopImuTracking();

  bool GetFiles(
      device_info_t *info, img_params_t *img_params, imu_params_t *imu_params,
      Version *spec_version = nullptr) const;
  bool SetFiles(
      device_info_t *info, img_params_t *img_params, imu_params_t *imu_params,
      Version *spec_version = nullptr);

 private:
  bool PuControlRange(
      Option option, int32_t *min, int32_t *max, int32_t *def) const;
  bool PuControlQuery(Option option, uvc::pu_query query, int32_t *value) const;

  bool XuControlRange(
      channel_t channel, uint8_t id, int32_t *min, int32_t *max,
      int32_t *def) const;
  bool XuControlRange(
      const uvc::xu &xu, uint8_t selector, uint8_t id, int32_t *min,
      int32_t *max, int32_t *def) const;

  bool XuControlQuery(
      channel_t channel, uvc::xu_query query, uint16_t size,
      uint8_t *data) const;
  bool XuControlQuery(
      const uvc::xu &xu, uint8_t selector, uvc::xu_query query, uint16_t size,
      uint8_t *data) const;

  bool XuCamCtrlQuery(uvc::xu_query query, uint16_t size, uint8_t *data) const;
  std::int32_t XuCamCtrlGet(Option option) const;
  void XuCamCtrlSet(Option option, std::int32_t value) const;

  bool XuHalfDuplexSet(Option option, xu_cmd_t cmd) const;

  bool XuImuWrite(const ImuReqPacket &req) const;
  bool XuImuRead(ImuResPacket *res) const;

  bool XuFileQuery(uvc::xu_query query, uint16_t size, uint8_t *data) const;

  control_info_t PuControlInfo(Option option) const;
  control_info_t XuControlInfo(Option option) const;

  std::shared_ptr<uvc::device> device_;
  std::shared_ptr<ChannelsAdapter> adapter_;

  std::map<Option, control_info_t> control_infos_;

  bool is_imu_tracking_;
  std::thread imu_track_thread_;
  volatile bool imu_track_stop_;

  std::uint32_t imu_sn_;
  imu_callback_t imu_callback_;
};

class ChannelsAdapter {
 public:
  virtual ~ChannelsAdapter() {}

  virtual std::set<Option> GetOptionSupports() = 0;

  virtual std::int32_t GetAccelRangeDefault() = 0;
  virtual std::vector<std::int32_t> GetAccelRangeValues() = 0;

  virtual std::int32_t GetGyroRangeDefault() = 0;
  virtual std::vector<std::int32_t> GetGyroRangeValues() = 0;

  virtual void GetImuResPacket(const std::uint8_t *data, ImuResPacket *res) = 0;

  virtual std::size_t GetImgParamsFromData(
      const std::uint8_t *data, const Version *version,
      Channels::img_params_t *img_params) = 0;
  virtual std::size_t SetImgParamsToData(
      const Channels::img_params_t *img_params, const Version *version,
      std::uint8_t *data) = 0;

  virtual std::size_t GetImuParamsFromData(
      const std::uint8_t *data, const Version *version,
      Channels::imu_params_t *imu_params);
  virtual std::size_t SetImuParamsToData(
      const Channels::imu_params_t *imu_params, const Version *version,
      std::uint8_t *data);
};

namespace bytes {

// from

template <typename T>
T _from_data(const std::uint8_t *data) {
  std::size_t size = sizeof(T) / sizeof(std::uint8_t);
  T value = 0;
  for (std::size_t i = 0; i < size; i++) {
    value |= data[i] << (8 * (size - i - 1));
  }
  return value;
}

template <>
inline double _from_data(const std::uint8_t *data) {
  return *(reinterpret_cast<const double *>(data));
}

std::string _from_data(const std::uint8_t *data, std::size_t count);

std::size_t from_data(Channels::device_info_t *info, const std::uint8_t *data);

std::size_t from_data(IntrinsicsPinhole *in, const std::uint8_t *data,
    const Version *spec_version);

std::size_t from_data(ImuIntrinsics *in, const std::uint8_t *data,
    const Version *spec_version);

std::size_t from_data(Extrinsics *ex, const std::uint8_t *data,
    const Version *spec_version);

// to

template <typename T>
std::size_t _to_data(T value, std::uint8_t *data) {
  std::size_t size = sizeof(T) / sizeof(std::uint8_t);
  for (std::size_t i = 0; i < size; i++) {
    data[i] = static_cast<std::uint8_t>((value >> (8 * (size - i - 1))) & 0xFF);
  }
  return size;
}

template <>
inline std::size_t _to_data(double value, std::uint8_t *data) {
  std::uint8_t *val = reinterpret_cast<std::uint8_t *>(&value);
  std::copy(val, val + 8, data);
  return 8;
}

std::size_t _to_data(std::string value, std::uint8_t *data, std::size_t count);

std::size_t to_data(const Channels::device_info_t *info, std::uint8_t *data,
    const Version *spec_version);

std::size_t to_data(const IntrinsicsPinhole *in, std::uint8_t *data,
    const Version *spec_version);

std::size_t to_data(const ImuIntrinsics *in, std::uint8_t *data,
    const Version *spec_version);

std::size_t to_data(const Extrinsics *ex, std::uint8_t *data,
    const Version *spec_version);

}  // namespace bytes

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_CHANNELS_CHANNELS_H_
