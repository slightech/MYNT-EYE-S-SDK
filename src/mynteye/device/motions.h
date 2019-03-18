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
#ifndef MYNTEYE_DEVICE_MOTIONS_H_
#define MYNTEYE_DEVICE_MOTIONS_H_
#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include "mynteye/mynteye.h"
#include "mynteye/device/callbacks.h"
#include "mynteye/device/types.h"

MYNTEYE_BEGIN_NAMESPACE

class Channels;

class Motions {
 public:
  using motion_data_t = device::MotionData;
  using motion_datas_t = std::vector<motion_data_t>;

  using motion_callback_t = device::MotionCallback;

  explicit Motions(std::shared_ptr<Channels> channels);
  ~Motions();

  void SetMotionCallback(motion_callback_t callback);
  void DoMotionTrack();

  void StartMotionTracking();
  void StopMotionTracking();

  void DisableMotionDatas();
  void EnableMotionDatas(std::size_t max_size);
  motion_datas_t GetMotionDatas();

  void SetMotionIntrinsics(const std::shared_ptr<MotionIntrinsics>& in);
  void EnableProcessMode(const std::int32_t& mode);

  void SetDeviceInfo(const std::shared_ptr<DeviceInfo>& in);

 private:
  void ProcImuAssembly(std::shared_ptr<ImuData> data) const;
  void ProcImuTempDrift(std::shared_ptr<ImuData> data) const;
  bool IsNullAssemblyOrTempDrift() const;

  std::shared_ptr<Channels> channels_;

  motion_callback_t motion_callback_;

  motion_datas_t motion_datas_;
  bool motion_datas_enabled_;
  std::size_t motion_datas_max_size_;

  bool is_imu_tracking;

  std::mutex mtx_datas_;

  int accel_range;
  int gyro_range;

  std::int32_t proc_mode_;
  std::shared_ptr<MotionIntrinsics> motion_intrinsics_;
  std::shared_ptr<DeviceInfo> device_info_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_MOTIONS_H_
