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
#ifndef MYNTEYE_INTERNAL_MOTIONS_H_  // NOLINT
#define MYNTEYE_INTERNAL_MOTIONS_H_
#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include "mynteye/callbacks.h"
#include "mynteye/mynteye.h"

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

  void EnableMotionDatas(std::size_t max_size);
  motion_datas_t GetMotionDatas();

 private:
  std::shared_ptr<Channels> channels_;

  motion_callback_t motion_callback_;

  motion_datas_t motion_datas_;
  bool motion_datas_enabled_;
  std::size_t motion_datas_max_size;

  bool is_imu_tracking;

  std::mutex mtx_datas_;

  int accel_range = 12;
  int gyro_range = 1000;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_MOTIONS_H_ NOLINT
