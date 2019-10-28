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
#ifndef MYNTEYE_DEVICE_CHANNEL_CHANNELS_H_
#define MYNTEYE_DEVICE_CHANNEL_CHANNELS_H_
#pragma once

#include <map>
#include <memory>
#include <set>
#include <thread>
#include <vector>

#include "mynteye/mynteye.h"
#include "mynteye/device/channel/def.h"
#include "mynteye/device/channel/file_channel.h"
#include "mynteye/device/device.h"
#include "mynteye/device/types.h"
#include "mynteye/uvc/uvc.h"

#define LIMMIT_CHECK_DORMANCY_THRESHOLD 100

MYNTEYE_BEGIN_NAMESPACE

namespace uvc {

struct device;
struct xu;

}  // namespace uvc

class ChannelsAdapter;

class MYNTEYE_API Channels {
 public:
  typedef struct ControlInfo {
    std::int32_t min;
    std::int32_t max;
    std::int32_t def;
  } control_info_t;

  typedef enum XuCmd {
    XU_CMD_ZDC = 0xE6,    // zero drift calibration
    XU_CMD_ERASE = 0xDE,  // erase chip
    XU_SYNC_TIMESTAMP = 0x200,  // sync timestamp
    XU_CMD_LAST
  } xu_cmd_t;

  using imu_callback_t = std::function<void(const ImuPacket2 &packet)>;

  using device_info_t = FileChannel::device_info_t;
  using img_params_t = FileChannel::img_params_t;
  using imu_params_t = FileChannel::imu_params_t;

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
  bool SetControlValue(const Option &option, std::uint64_t value);

  bool RunControlAction(const Option &option) const;

  void SetImuCallback(imu_callback_t callback);
  void DoImuTrack();
  void DoImuTrack1();
  void DoImuTrack1WithTimeLimmitFix();
  void DoImuTrack2();

  void StartImuTracking(imu_callback_t callback = nullptr);
  void StopImuTracking();

  bool GetFiles(
      device_info_t *info, img_params_t *img_params, imu_params_t *imu_params);
  bool SetFiles(
      device_info_t *info, img_params_t *img_params, imu_params_t *imu_params);
  inline bool IsImuProtocol2() const { return is_imu_proto2_; }
  inline bool IsS2() const { return is_s2_; }
  inline void EnableImuCorrespondence(bool is_enable) {
    enable_imu_correspondence = is_enable;
  }
  inline uint64_t timestamp_compensate(uint32_t timestamp32) {
    return 0;
  }
  inline void SetAcceRange(const int & value) {accel_range = value;}
  inline void SetGyroRange(const int & value) {gyro_range = value;}

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
  bool XuHalfDuplexSet(Option option, std::uint64_t value) const;

  bool XuImuWrite(const ImuReqPacket &req) const;
  bool XuImuWrite(const ImuReqPacket2 &req) const;
  bool XuImuRead(ImuResPacket *res) const;
  bool XuImuRead(ImuResPacket2 *res) const;

  bool XuFileQuery(uvc::xu_query query, uint16_t size, uint8_t *data) const;
  void CheckTimeStampLimmit(mynteye::ImuPacket2 &packet);  // NOLINT

  control_info_t PuControlInfo(Option option) const;
  control_info_t XuControlInfo(Option option) const;

  std::shared_ptr<uvc::device> device_;
  std::shared_ptr<ChannelsAdapter> adapter_;

  FileChannel file_channel_;

  std::map<Option, control_info_t> control_infos_;

  bool is_imu_tracking_;
  bool is_imu_proto2_;
  bool is_s2_;
  bool enable_imu_correspondence;
  std::thread imu_track_thread_;
  volatile bool imu_track_stop_;
  int accel_range;
  int gyro_range;
  std::uint32_t imu_sn_;
  imu_callback_t imu_callback_;
  std::shared_ptr<device_info_t> dev_info_;
  uint32_t timestamp_compensate_;
  uint16_t is_nearly_before_timestamp_limmit_;
  uint64_t current_datum_;
};

class ChannelsAdapter {
 public:
  explicit ChannelsAdapter(const Model &model);
  virtual ~ChannelsAdapter();

  virtual std::set<Option> GetOptionSupports();
  virtual std::set<Resolution> GetResolutionSupports();

  virtual std::int32_t GetAccelRangeDefault() = 0;
  virtual std::vector<std::int32_t> GetAccelRangeValues() = 0;

  virtual std::int32_t GetGyroRangeDefault() = 0;
  virtual std::vector<std::int32_t> GetGyroRangeValues() = 0;

  virtual void GetImuResPacket(const std::uint8_t *data, ImuResPacket *res) = 0;
  virtual void GetImuResPacket2(const std::uint8_t *data,
      ImuResPacket2 *res, bool is_correspondence_on) = 0;

 protected:
  Model model_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_CHANNEL_CHANNELS_H_
